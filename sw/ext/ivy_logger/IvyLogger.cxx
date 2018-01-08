/**
 * This application logs all incoming Ivy bus messages
 */

#include <iostream> // for io
#include <thread> // for threads
#include <chrono> // for thread sleep and time count
#include <string> // for string handling
#include <unistd.h> // for getopt
#include <time.h>   // time_t, struct tm, time, localtime
#include <fstream> // Stream class to both read and write from/to files.
#include <sstream> // string operations
#include <string.h> // strings
#include <vector> // for vectors
#include <sstream> // to intelligently format a string
#include <iomanip> // to pretty print string

#include "Ivycpp.h"
#include "IvyApplication.h"

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;

using namespace std;

class IvyLogger: public IvyApplicationCallback, public IvyMessageCallback
{
public:
  Ivy *bus;

  void Start();
  void OnApplicationConnected(IvyApplication *app);
  void OnApplicationDisconnected(IvyApplication *app);
  void OnApplicationCongestion(IvyApplication *app);
  void OnApplicationDecongestion(IvyApplication *app);
  void OnApplicationFifoFull(IvyApplication *app);
  void OnMessage(IvyApplication *app, int argc, const char **argv);

  static std::vector<std::string> parse_msg(const char **argv);

  static void ivy_thread(IvyLogger *test);

  IvyLogger(char *domain, bool debug, char* name);
  IvyLogger(char *domain, bool debug);
  IvyLogger(char *domain);
  IvyLogger();
  void init();

  static string getTimestamp(struct tm* timeinfo);
  static string getLogName(struct tm* timeinfo);

private:
  static void ivyAppConnCb(IvyApplication *app)
  {
  }
  ;
  static void ivyAppDiscConnCb(IvyApplication *app)
  {
  }
  ;

  const char *bus_domain_;

  time_t rawtime_;
  struct tm* timeinfo_;
  float sec_since_startup_;
  Clock::time_point t0;

  string name_ = "ivy-logger";  // ivy node name
  bool debug_;  // are we in debug mode?
};

void IvyLogger::init()
{
  sec_since_startup_ = 0.0;
  time(&rawtime_);
  timeinfo_ = localtime(&rawtime_);
  // update clock
  t0 = Clock::now();
}

string IvyLogger::getLogName(struct tm* timeinfo)
{
  int year = timeinfo->tm_year + 1900;  // YYYY
  std::ostringstream oss;
  oss << year << "_";

  // month are [0-11] by default
  int mon = timeinfo->tm_mon + 1;
  oss << setfill('0') << setw(2) << mon << "_";

  int day = timeinfo->tm_mday;
  oss << setfill('0') << setw(2) << day << "__";

  int hour = timeinfo->tm_hour;
  oss << setfill('0') << setw(2) << hour << "_";

  int min = timeinfo->tm_min;
  oss << setfill('0') << setw(2) << min << "_";

  int sec = timeinfo->tm_sec;
  oss << setfill('0') << setw(2) << sec << "_logger.log";

  // YY_MM_DD__HH_MM_SS
  return oss.str();
}

string IvyLogger::getTimestamp(struct tm* timeinfo)
{
  string months[] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  };
  std::ostringstream oss;

  int mon = timeinfo->tm_mon;
  oss << months[mon] << " ";

  int day = timeinfo->tm_mday;
  oss << setfill('0') << setw(2) << day << " ";

  int hour = timeinfo->tm_hour;
  oss << setfill('0') << setw(2) << hour << ":";

  int min = timeinfo->tm_min;
  oss << setfill('0') << setw(2) << min << ":";

  int sec = timeinfo->tm_sec;
  oss << setfill('0') << setw(2) << sec;

  // MMM DD HH:MM:SS
  return oss.str();
}

IvyLogger::IvyLogger(char *domain, bool debug, char* name)
{
  init();
  if (name != NULL) {
    this->name_ = string(name);
  }
  debug_ = debug;
  bus_domain_ = domain;
  bus = new Ivy(this->name_.c_str(), (this->name_ + " READY").c_str(),
      BUS_APPLICATION_CALLBACK(ivyAppConnCb, ivyAppDiscConnCb), false);
}

IvyLogger::IvyLogger(char *domain, bool debug)
{
  init();
  debug_ = debug;
  bus_domain_ = domain;
  bus = new Ivy(this->name_.c_str(), (this->name_ + " READY").c_str(),
      BUS_APPLICATION_CALLBACK(ivyAppConnCb, ivyAppDiscConnCb), false);
}

IvyLogger::IvyLogger(char *domain)
{
  init();
  debug_ = false;
  bus_domain_ = domain;
  bus = new Ivy(this->name_.c_str(), (this->name_ + " READY").c_str(),
      BUS_APPLICATION_CALLBACK(ivyAppConnCb, ivyAppDiscConnCb), false);
}

IvyLogger::IvyLogger()
{
  init();
  debug_ = false;
  bus_domain_ = NULL;
  bus = new Ivy(this->name_.c_str(), (this->name_ + " READY").c_str(),
      BUS_APPLICATION_CALLBACK(ivyAppConnCb, ivyAppDiscConnCb), false);
}

void IvyLogger::Start()
{
  // bind all messages
  bus->BindMsg("(.*)", this);

  // start bus
  bus->start(bus_domain_);

  // enter main loop
  bus->ivyMainLoop();
}

/**
 * Parse the incoming messages
 */
std::vector<std::string> IvyLogger::parse_msg(const char **argv)
{
  std::string s(argv[0]);
  std::string delimiter = " ";
  std::vector<std::string> argList;

  size_t pos = 0;
  std::string token;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    token = s.substr(0, pos);
    std::string tmp = token;
    argList.push_back(tmp);
    s.erase(0, pos + delimiter.length());
  }
  argList.push_back(s);

#if DEBUG
  cout << "Original message: " << endl;
  cout << argv[0] << endl;

  for(std::vector<string>::iterator it = argList.begin(); it != argList.end(); ++it) {
    cout << *it << " ";
  }
  cout << endl;
#endif

  return argList;
}

/**
 * Logging all incoming messages
 */
void IvyLogger::OnMessage(IvyApplication *app, int argc, const char **argv)
{
  std::vector<std::string> argList = parse_msg(argv);

  if (argList.size() > 1) {
    // log data
    static ofstream logfile;

    // YY_MM_DD__HH_MM_SS.log
    static std::string fname = "../../../var/logs/" + getLogName(this->timeinfo_);
    logfile.open(fname.c_str(), std::ios_base::app);

    static bool show = true;
    if (show) {
      cout << "Logging to: " << fname << endl;
      string timestamp = getTimestamp(this->timeinfo_);
      logfile << timestamp << " INFO " << "ivy bus logger " << endl;
      logfile << timestamp << " INFO ivy_bus:" << this->bus_domain_ << endl;
      show = false;
    }

    // get current time stamp
    time_t t = time(0);   // get time now
    struct tm *now = localtime(&t);
    string timestamp = getTimestamp(now);

    // write message
    logfile << timestamp << " " << " INFO " << "ivy_callback ";
    logfile << argList[0] << " ";
    for (int i = 1; i < argList.size() - 1; i++) {
      logfile << argList[i] << " ";
    }
    logfile << argList[argList.size() - 1] << endl;

    logfile.close();
  }
}

void IvyLogger::OnApplicationConnected(IvyApplication *app)
{
  const char *appname;
  const char *host;
  appname = app->GetName();
  host = app->GetHost();
  printf("%s connected from %s\n", appname, host);
}

void IvyLogger::OnApplicationDisconnected(IvyApplication *app)
{
  const char *appname;
  const char *host;
  appname = app->GetName();
  host = app->GetHost();
  printf("%s disconnected from %s\n", appname, host);
}

void IvyLogger::OnApplicationCongestion(IvyApplication *app)
{
  std::cerr << "Ivy Congestion notififation\n";
}
void IvyLogger::OnApplicationDecongestion(IvyApplication *app)
{
  std::cerr << "Ivy Decongestion notififation\n";
}
void IvyLogger::OnApplicationFifoFull(IvyApplication *app)
{
  std::cerr << "Ivy FIFO Full  notififation : MESSAGE WILL BE LOST\n";
}

/**
 * This thread starts the Ivy Bus and the enters the IvyMainLoop
 */
void IvyLogger::ivy_thread(IvyLogger *test)
{
  test->Start();
}

void showhelpinfo(char *s)
{
  cout << "Usage:   " << s << " [-option] [argument]" << endl;
  cout << "option:  " << "-h  show help information" << endl;
  cout << "         " << "-b ivy bus (default is 127.255.255.255:2010)" << endl;
  cout << "         "
      << "-d simulation mode (default is false, use 'true' or '1')" << endl;
  cout << "         " << "-n name (default is \"ivy-logger\")" << endl;
  cout << "example: " << s << " -b 10.0.0.255:2010" << endl;
}

int main(int argc, char** argv)
{
  char tmp;
  char* ivy_bus = NULL;
  bool debug = false;
  char* name = NULL;

  while ((tmp = getopt(argc, argv, "hb:d:n:")) != -1) {
    switch (tmp) {
      /*option h show the help information*/
      case 'h':
        showhelpinfo(argv[0]);
        exit(1);
        break;
      case 'b':
        ivy_bus = optarg;
        cout << "-b " << ivy_bus << endl;
        break;
      case 'd':
        // check values of debug argument
        if (!strcmp("true", optarg)) {
          // equals true
          debug = true;
        }
        if (!strcmp("1", optarg)) {
          debug = true;
        }
        break;
      case 'n':
        name = optarg;
        break;
        /*do nothing on default*/
      default:
        break;
    }
  }

  IvyLogger logger(ivy_bus, debug, name);

  //Launch a thread
  std::thread t1(IvyLogger::ivy_thread, &logger);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  //Wait for the ivy_thread to end
  t1.join();

  return 0;
}
