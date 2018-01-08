#![feature(rustc_private)]
extern crate log; // logging
#[macro_use]
extern crate lazy_static; // global variables
#[macro_use]
extern crate slog; // logging
#[macro_use]
extern crate slog_scope; // logging
extern crate slog_stdlog; // logging
extern crate slog_term; // logging
extern crate chrono; // datetime
extern crate ivyrust; // ivy bus
extern crate pprzlink; // pprzlink
extern crate clap; // arguments

use pprzlink::parser;
use pprzlink::parser::{PprzDictionary, PprzMsgClassID, PprzMessage};
use pprzlink::transport::PprzTransport;

use ivyrust::*;
use clap::{Arg, App};
use std::{thread, time};
use std::env;
use std::error::Error;
use std::fs::File;
use std::sync::{Arc, Mutex}; // shared dictionary
use std::net::UdpSocket; // networking
use std::fs::OpenOptions; // logging
use slog::Drain; // logging
use chrono::prelude::*; // log filename

lazy_static! {
    static ref MSG_QUEUE: Mutex<Vec<PprzMessage>> = Mutex::new(vec![]);
    static ref DICTIONARY: Mutex<Vec<PprzDictionary>> = Mutex::new(vec![]);
}

/// Main IVY loop
///
/// This thread only launchees `IvyMainLoop()` and loops forever
/// Uses the optional argument specifying a non-default bus address
fn thread_ivy_main(ivy_bus: String) -> Result<(), Box<Error>> {
    ivyrust::ivy_init(String::from("Link"), String::from("Ready"));
    if !ivy_bus.is_empty() {
        ivyrust::ivy_start(Some(ivy_bus));
    } else {
        ivyrust::ivy_start(None);
    }
    ivyrust::ivy_main_loop()
}


/// Datalink listening thread
///
/// Listen for datalink messages and push them on ivy bus
/// Listens on `udp_port` on local interface
fn thread_datalink(port: &str, dictionary: Arc<Mutex<PprzDictionary>>) -> Result<(), Box<Error>> {
    let name = "thread_datalink";
    let addr = String::from("127.0.0.1");
    let addr = addr + ":" + port;
    let socket = UdpSocket::bind(&addr)?;

    let mut buf = vec![0; 1024];
    let mut rx = PprzTransport::new();

    println!("{} at {}", name, addr);
    loop {
        let (len, _) = socket.recv_from(&mut buf)?;
        for idx in 0..len {
            if rx.parse_byte(buf[idx]) {
                let dict = dictionary.lock().unwrap();
                let msg_name = dict.get_msg_name(PprzMsgClassID::Datalink, rx.buf[1])
                    .unwrap();
                let mut msg = dict.find_msg_by_name(&msg_name).unwrap();

                // update message fields with real values
                msg.update(&rx.buf);

                // send the message
                info!("{}: {}", name, msg.to_string().unwrap());
                ivyrust::ivy_send_msg(msg.to_string().unwrap());
            }
        }
    }
}


/// Telemetry passing thread
///
/// Upon receiving a new message from Ivy bus,
/// it sends it over on `udp_uplink_port`
fn thread_telemetry(port: &str, _dictionary: Arc<Mutex<PprzDictionary>>) -> Result<(), Box<Error>> {
    let name = "thread_telemetry";
    let addr = String::from("127.0.0.1:33255"); // TODO: fix ports
    let socket = UdpSocket::bind(&addr)?;

    let remote_addr = String::from("127.0.0.1");
    let remote_addr = remote_addr + ":" + port;

    println!("{} at {}", name, addr);
    loop {
        {
            // check for new messages in the message queue, super ugly
            let mut lock = MSG_QUEUE.lock();
            if let Ok(ref mut msg_queue) = lock {
                while !msg_queue.is_empty() {
                    // get a message from the front of the queue
                    let new_msg = msg_queue.pop().unwrap();

                    // get a transort
                    let mut tx = PprzTransport::new();

                    info!("{}: {}", name, new_msg.to_string().unwrap());

                    // construct a message from the transport
                    tx.construct_pprz_msg(&new_msg.to_bytes());

                    // send to remote address
                    socket.send_to(&tx.buf, &remote_addr)?;
                }
            }
        }

        // sleep
        thread::sleep(time::Duration::from_millis(50));
    }
}




/// This global callback is just a cludge for now
fn global_ivy_callback(mut data: Vec<String>) {
    let data = &(data.pop().unwrap());
    let mut lock = DICTIONARY.try_lock();
    let name = "ivy callback";

    if let Ok(ref mut dictionary_vector) = lock {
        if !dictionary_vector.is_empty() {
            let dictionary = &dictionary_vector[0];
            let msgs = dictionary
                .get_msgs(PprzMsgClassID::Telemetry)
                .unwrap()
                .messages;

            let values: Vec<&str> = data.split(|c| c == ' ' || c == ',').collect();
            if values.len() >= 2 {
                // iterate over messages
                for mut msg in msgs {
                    if values[1] == &msg.name {
                        // parse the message and push it into the message queue
                        msg.set_sender(values[0].parse::<u8>().unwrap());
                        msg.update_from_string(&values);
                        info!("{}: {}", name, msg.to_string().unwrap());

                        // if found, update the global msg
                        let mut msg_lock = MSG_QUEUE.lock();
                        if let Ok(ref mut msg_vector) = msg_lock {
                            // append at the end of vector
                            msg_vector.push(msg);
                        }
                        break;
                    }
                }
            }
        }
    }
}

fn get_log_filename() -> String {
    let time: DateTime<Local> = Local::now();
    let log_path = time.year().to_string() + "_" + &format!("{:>02}", time.month()) + "_" +
        &format!("{:>02}", time.day()) + "__" + "_" +
        &format!("{:>02}", time.hour()) + "_" +
        &format!("{:>02}", time.minute()) + "_" +
        &format!("{:>02}", time.second()) + "_forwarder.log";
    log_path
}


fn main() {
    // open log file

    let log_path = String::from("../../../var/logs/") + &get_log_filename();
    let file = OpenOptions::new()
        .create(true)
        .write(true)
        .truncate(true)
        .open(log_path)
        .unwrap();

    // create logger
    let decorator = slog_term::PlainSyncDecorator::new(file);
    let drain = slog_term::FullFormat::new(decorator).build().fuse();
    let logger = slog::Logger::root(drain, o!());

    // slog_stdlog uses the logger from slog_scope, so set a logger there
    let _guard = slog_scope::set_global_logger(logger);

    // register slog_stdlog as the log handler with the log crate
    slog_stdlog::init().unwrap();

    // Construct command line arguments
    let matches = App::new(
        "Link forwarder.\n
    Forward telemetry from Ivy bus over UDP port to ground,and listen to datalink messages
    coming to UDP_uplink port and relay those on Ivy bus",
    ).version("0.1")
        .arg(
            Arg::with_name("ivy_bus")
                .short("b")
                .value_name("ivy_bus")
                .help("Default is 127.255.255.255:2010")
                .takes_value(true),
        )
        .arg(
            Arg::with_name("udp_port")
                .short("d")
                .value_name("UDP port")
                .help("Default is 4242")
                .takes_value(true),
        )
        .arg(
            Arg::with_name("udp_uplink_port")
                .short("u")
                .value_name("UDP uplink port")
                .help("Default is 4243")
                .takes_value(true),
        )
        .get_matches();

    let ivy_bus = String::from(matches.value_of("ivy_bus").unwrap_or(
        "127.255.255.255:2010",
    ));
    println!("Value for ivy_bus: {}", ivy_bus);

    let udp_port = String::from(matches.value_of("udp_port").unwrap_or("4242"));
    println!("Value for udp_port: {}", udp_port);

    let udp_uplink_port = String::from(matches.value_of("udp_uplink_port").unwrap_or("4243"));
    println!("Value for udp_uplink_port: {}", udp_uplink_port);

    let pprz_root = match env::var("PAPARAZZI_SRC") {
        Ok(var) => var,
        Err(e) => {
            println!("Error getting PAPARAZZI_SRC environment variable: {}", e);
            return;
        }
    };

    info!("Rustlink-forwarder logger");
    info!(
        "ivy bus: {}, UDP port: {}, UDP uplink port: {}",
        ivy_bus,
        udp_port,
        udp_uplink_port
    );

    // spin the main IVY loop
    let _ = thread::spawn(move || if let Err(e) = thread_ivy_main(ivy_bus) {
        println!("Error starting ivy thread: {}", e);
    } else {
        println!("Ivy thread finished");
    });

    // ugly ugly hack to get the ivy callback working
    let xml_file = pprz_root.clone() + "/sw/ext/pprzlink/message_definitions/v1.0/messages.xml";
    let file = File::open(xml_file.clone()).unwrap();
    DICTIONARY.lock().unwrap().push(
        parser::build_dictionary(file),
    );

    // prepare the dictionary
    let xml_file = pprz_root + "/sw/ext/pprzlink/message_definitions/v1.0/messages.xml";
    let file = File::open(xml_file.clone()).unwrap();
    let dictionary = Arc::new(Mutex::new(parser::build_dictionary(file)));

    // spin listening thread
    let dict = dictionary.clone();
    let t_datalink = thread::spawn(move || if let Err(e) = thread_datalink(&udp_port, dict) {
        println!("Error starting datalink thread: {}", e);
    } else {
        println!("Datalink thread finished");
    });

    // spin sending thread
    let dict = dictionary.clone();
    let t_telemetry =
        thread::spawn(move || if let Err(e) = thread_telemetry(&udp_uplink_port, dict) {
            println!("Error starting telemetry thread: {}", e);
        } else {
            println!("Datalink telemetry finished");
        });

    // bind global callback
    let _ = ivy_bind_msg(global_ivy_callback, String::from("(.*)"));

    t_datalink.join().expect(
        "Error waiting for datalink thread to finish",
    );
    t_telemetry.join().expect(
        "Error waiting for telemetry thread to finish",
    );
    println!("Done")
}
