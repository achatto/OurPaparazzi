(*
 * XML preprocessing of messages.xml for aiborne middleware ABI
 *
 * Copyright (C) 2011 ENAC, Gautier Hattenberger
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 *)

open Printf

type _type = string
type _name = string

type field = _name * _type

type fields = field list

type message = {
  name : string;
  update : string;
  fields : fields
}

module Syntax = struct
  (** Translates a "message" XML element into a value of the 'message' type *)
  let struct_of_xml = fun update xml ->
    let name = ExtXml.attrib xml "name"
    and fields =
      List.map
        (fun field ->
          let _name = ExtXml.attrib field "name"
          and _type = ExtXml.attrib field "type" in
          (_name, _type))
        (Xml.children xml) in
    { name = name; update = update; fields = fields }

  (** Translates XML message file into a list of messages *)
  let read = fun filename ->
    let xml = Xml.parse_file filename in
    let sync = try Xml.children (ExtXml.child xml "messages") with _ -> [] in
    let async = try Xml.children (ExtXml.child xml "async") with _ -> [] in
    (*let sync, async = List.partition (fun m ->
      let u = ExtXml.attrib_or_default m "update" "sync" in
      match u with
      | "sync" -> true
      | "async" -> false
      | _ -> failwith (sprintf "Gen_abi: invalid update type '%s'" u)
      ) (Xml.children xml) in*)
    let msgs_sync = List.map (struct_of_xml "sync") sync in
    let msgs_async = List.map (struct_of_xml "async") async in
    msgs_sync, msgs_async
end (* module Suntax *)


(** Pretty printer *)
module Gen_onboard = struct
  (* Print message IDs and return the highest value *)
  let print_message_id = fun h messages ->
    let highest_id = ref 0 in
    Printf.fprintf h "\n/* Messages IDs */\n";
    List.iter (fun msg ->
      Printf.fprintf h "#define ABI_%s_ID %d\n" (String.capitalize msg.name) !highest_id;
      incr highest_id
    ) messages;
    !highest_id

  (* Print structure array *)
  let print_struct = fun h size update ->
    Printf.fprintf h "\n/* Array and linked list structure %s */\n" update;
    Printf.fprintf h "#define ABI_MESSAGE_%s_NB %d\n\n" (String.uppercase update) size;
    Printf.fprintf h "ABI_EXTERN abi_%s_list abi_queues_%s[ABI_MESSAGE_%s_NB];\n" update update (String.uppercase update)

  (* Print arguments' function from fields *)
  let print_args = fun h fields ->
    let rec args = fun h l ->
      match l with
          [] -> Printf.fprintf h ")"
        | [(n,t)] -> Printf.fprintf h ", const %s * %s)" t n
        | (n,t)::l' -> Printf.fprintf h ", const %s * %s" t n; args h l'
    in
    Printf.fprintf h "(uint8_t sender_id";
    args h fields

  (* Print callbacks prototypes for all messages *)
  let print_callbacks = fun h messages ->
    Printf.fprintf h "\n/* Callbacks */\n";
    List.iter (fun msg ->
      Printf.fprintf h "typedef void (*abi_callback%s)" (String.capitalize msg.name);
      print_args h msg.fields;
      Printf.fprintf h ";\n";
    ) messages

  (* Print a sync bind function *)
  let print_sync_msg_bind = fun h msg ->
    let name = String.capitalize msg.name in
    Printf.fprintf h "\nstatic inline void AbiBindMsg%s(uint8_t sender_id, abi_event * ev, abi_callback%s cb) {\n" name name;
    Printf.fprintf h "  ev->id = sender_id;\n";
    Printf.fprintf h "  ev->cb = (abi_callback)cb;\n";
    Printf.fprintf h "  ABI_PREPEND(abi_queues_sync[ABI_%s_ID], ev);\n" name;
    Printf.fprintf h "}\n"

  (* Print a sync send function *)
  let print_sync_msg_send = fun h msg ->
    (* print arguments *)
    let rec args = fun h l ->
      match l with
          [] -> Printf.fprintf h ");\n"
        | [(n,_)] -> Printf.fprintf h ", %s);\n" n
        | (n,_)::l' -> Printf.fprintf h ", %s" n; args h l'
    in
    let name = String.capitalize msg.name in
    Printf.fprintf h "\nstatic inline void AbiSendMsg%s" name;
    print_args h msg.fields;
    Printf.fprintf h " {\n";
    Printf.fprintf h "  abi_event* e;\n";
    Printf.fprintf h "  ABI_FOREACH(abi_queues_sync[ABI_%s_ID], e) {\n" name;
    Printf.fprintf h "    if (e->id == ABI_BROADCAST || e->id == sender_id) {\n";
    Printf.fprintf h "      abi_callback%s cb = (abi_callback%s)(e->cb);\n" name name;
    Printf.fprintf h "      cb(sender_id";
    args h msg.fields;
    Printf.fprintf h "    };\n";
    Printf.fprintf h "  };\n";
    Printf.fprintf h "};\n"

  (* Print a async bind function *)
  let print_async_msg_bind = fun h msg ->
    let name = String.capitalize msg.name in
    Printf.fprintf h "\nstatic inline void AbiBindAsyncMsg%s(abi_event * ev, abi_callback%s cb) {\n" name name;
    Printf.fprintf h "  ev->cb = (abi_callback)cb;\n";
    Printf.fprintf h "  ABI_PREPEND(abi_queues_async[ABI_%s_ID].ev, ev);\n" name;
    Printf.fprintf h "}\n"

  (* Print a async send function *)
  let print_async_msg_send = fun h msg ->
    let name = String.capitalize msg.name in
    Printf.fprintf h "\nstatic inline void AbiRaiseAsyncMsg%s(void)" name;
    Printf.fprintf h " {\n";
    Printf.fprintf h "  abi_queues_async[ABI_%s_ID].updated = TRUE;\n" name;
    Printf.fprintf h "};\n"

  (* Print async check and call function *)
  let print_async_check = fun h messages ->
    (* print arguments *)
    let rec args = fun h l ->
      match l with
          [] -> Printf.fprintf h ");\n"
        | [(n,_)] -> Printf.fprintf h ", %s);\n" n
        | (n,_)::l' -> Printf.fprintf h ", %s" n; args h l'
    in
    Printf.fprintf h "\nstatic inline void AbiCheckAsyncMsg(void) {\n";
    List.iter (fun msg ->
      let name = String.capitalize msg.name in
      Printf.fprintf h "  if (abi_queues_async[ABI_%s_ID].updated) {\n" name;
      Printf.fprintf h "    abi_event* e;\n";
      Printf.fprintf h "    ABI_FOREACH(abi_queues_async[ABI_%s_ID].ev, e) {\n" name;
      Printf.fprintf h "      abi_callback%s cb = (abi_callback%s)(e->cb);\n" name name;
      Printf.fprintf h "      cb(ABI_BROADCAST";
      args h msg.fields;
      Printf.fprintf h "    };\n";
      Printf.fprintf h "    abi_queues_async[ABI_%s_ID].updated = FALSE;\n" name;
      Printf.fprintf h "  };\n"
    ) messages;
    Printf.fprintf h "};\n"

  (* Print sync bind and send functions for all messages *)
  let print_sync_bind_send = fun h messages ->
    Printf.fprintf h "\n/* Sync Bind and Send functions */\n";
    List.iter (fun msg ->
      print_sync_msg_bind h msg;
      print_sync_msg_send h msg
    ) messages

  (* Print async bind and send functions for all messages *)
  let print_async_bind_send = fun h messages ->
    Printf.fprintf h "\n/* Async Bind and Send functions */\n";
    List.iter (fun msg ->
      print_async_msg_bind h msg;
      print_async_msg_send h msg
    ) messages

end (* module Gen_onboard *)


(********************* Main **************************************************)
let () =
  if Array.length Sys.argv <> 3 then begin
    failwith (sprintf "Usage: %s <.xml file> <class_name>" Sys.argv.(0))
  end;

  let filename = Sys.argv.(1) in

  try
    let h = stdout in

    (** Read and store messages *)
    let sync, async = Syntax.read filename in

    (** Print file header *)
    Printf.fprintf h "/* Automatically generated from %s */\n" filename;
    Printf.fprintf h "/* Please DO NOT EDIT */\n\n";
    Printf.fprintf h "/* Onboard middleware library ABI\n";
    Printf.fprintf h " */\n\n";
    Printf.fprintf h "#ifndef ABI_MESSAGES_H\n";
    Printf.fprintf h "#define ABI_MESSAGES_H\n\n";
    Printf.fprintf h "#include \"subsystems/abi_common.h\"\n";

    (** Print Sync messages IDs *)
    let highest_id = Gen_onboard.print_message_id h sync in
    (** Print general structure definition *)
    Gen_onboard.print_struct h highest_id "sync";
    (** Print Messages callbacks definition *)
    Gen_onboard.print_callbacks h sync;
    (** Print Bind and Send functions for all sync messages *)
    Gen_onboard.print_sync_bind_send h sync;

    (** Print Async messages IDs *)
    let highest_id = Gen_onboard.print_message_id h async in
    (** Print general structure definition *)
    Gen_onboard.print_struct h highest_id "async";
    (** Print Messages callbacks definition *)
    Gen_onboard.print_callbacks h async;
    (** Print Bind and Send functions for all async messages *)
    Gen_onboard.print_async_bind_send h async;
    (** Print Async messages check function *)
    Gen_onboard.print_async_check h async;

    Printf.fprintf h "\n#endif // ABI_MESSAGES_H\n"
  with
      Xml.Error (msg, pos) -> failwith (sprintf "%s:%d : %s\n" filename (Xml.line pos) (Xml.error_msg msg))
