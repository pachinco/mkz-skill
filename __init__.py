
# general imports
import os
import sys
import ast
import json
from time import strftime, localtime
from datetime import datetime
from pathlib import Path
#import threading

# ROS imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Mycroft imports
from adapt.intent import IntentBuilder
from mycroft import MycroftSkill, intent_file_handler
from mycroft import MycroftSkill, intent_handler
from mycroft.util import play_wav
from mycroft.skills import resting_screen_handler
from mycroft.skills.context import adds_context, removes_context

class RosBridge(Node):
    def __init__(self, skill):
        super().__init__('ros')
        self.skill = skill
        self.log = skill.log
        self.sub_cmd = self.create_subscription(String, 'hmi_cmd', self.sub_cmd_rcv, 5)
        self.sub_cmd  # prevent unused variable warning
        self.sub_ctrl = self.create_subscription(String, 'hmi_ctrl', self.sub_ctrl_rcv, 5)
        self.sub_ctrl  # prevent unused variable warning
        self.pub_ctrl = self.create_publisher(String, 'hmi_ctrl', 5)
        self.pub_cmd = self.create_publisher(String, 'hmi_cmd', 5)
        self.pub_hmi = self.create_publisher(String, 'hmi', 5)

    def sub_cmd_rcv(self, msg):
        self.log.info('ros.sub_cmd_rcv: "%s"' % msg.data)
        c = json.loads(msg.data)
        #c = json.loads(msg.data.replace("'", '"'))
        for k,v in c.items():
            self.log.info('ros.sub_cmd_rcv: %s:%s' % (k, v))
            if k == "ask":
                if not self.skill.voice_ask_options(v):
                    break
            elif k == "add":
                self.skill.control_add(v)
            elif k == "speak":
                self.skill.speak(v)
            elif k == "dialog":
                self.skill.speak_dialog(v)
            elif k == "cancel":
                self.skill.voice_ask_cancel(v)

    def sub_ctrl_rcv(self, msg):
        self.log.info('ros.sub_ctrl_rcv: "%s"' % msg.data)
        c = json.loads(msg.data)
        #c = json.loads(msg.data.replace("'", '"'))
        for k,v in c.items():
            self.log.info('ros.sub_ctrl_rcv: %s:%s' % (k, v))
            if self.skill.ask:
                self.skill.voice_ask_cancel(k)
                #self.log.info('ros.sub_ctrl_rcv: signal %s:%s' % (k, v))
            if k in self.skill.signal:
                self.skill.change_value(k, v)

    def rclpy_spin_once(self):
        rclpy.spin_once(self, timeout_sec=0)

    def pub_ctrl_snd(self, msg):
        self.log.info('ros.pub_ctrl_snd: %s', msg.data)
        self.pub_ctrl.publish(msg)
        self.rclpy_spin_once()

    def pub_cmd_snd(self, msg):
        self.log.info('ros.pub_cmd_snd: %s', msg.data)
        self.pub_cmd.publish(msg)
        self.rclpy_spin_once()

    def pub_hmi_snd(self, msg):
        self.log.info('ros.pub_hmi_snd: %s', msg.data)
        self.pub_hmi.publish(msg)
        self.rclpy_spin_once()

    def send_cmd_data(self, data):
        msg = String()
        msg.data = json.dumps(data, separators=(',', ':'))
        self.log.info('ros.send_cmd_data: %s', msg.data)
        self.pub_cmd_snd(msg)

    def send_ctrl_data(self, data):
        msg = String()
        msg.data = json.dumps(data, separators=(',', ':'))
        self.log.info('ros.send_ctrl_data: %s', msg.data)
        self.pub_ctrl_snd(msg)

    def send_hmi_data(self, data):
        msg = String()
        msg.data = json.dumps(data, separators=(',', ':'))
        self.log.info('ros.send_hmi_data: %s', msg.data)
        self.pub_hmi_snd(msg)

class Mkz(MycroftSkill):
    def __init__(self):
        MycroftSkill.__init__(self)
        self.greeting = True
        #self.sound_file_path = Path(__file__).parent.joinpath("sounds", "mkz-welcome-chime2.wav")

    def rclpy_init(self):
        self.log.info("skill.rclpy_init");
        self._args=sys.argv
        self.log.info(self._args)
        self._context = rclpy.context.Context()
        self.log.info(self._context)
        if not rclpy.utilities.ok():
            rclpy.init(args=self._args)

    def rclpy_shutdown(self):
        if self.ros is not None:
            self.ros.destroy_node()
        if rclpy.utilities.ok():
            rclpy.shutdown()

    def rclpy_activate(self):
        self.schedule_repeating_event(self.ros.rclpy_spin_once, None, 0.1)

    def keep_alive_activate(self):
        self.schedule_repeating_event(self.keep_active, None, 60)

    def initialize(self):
        self.log.info("skill.initialize");
        self.uiIdxKeys = {"none": 0, "map": 2, "maps": 3, "addresses": 4, "address": 4, "rolodex": 4, "locations": 4, "status": 8, "diagnostics": 8, "control": 16, "controls": 16, "media": 32, "music": 32, "weather": 64, "news": 128}
        self.uiIdxStickyKeys = ["sticky", "hold", "permanent"]
        self.ui={}
        self.ui["uiIdx"] = 0
        self.ui["uiIdx_Sticky"] = 0
        self.ask = {}
        self.ask_cancel = ("cancel", "shut up", "stop")
        self.ask_converse = False
        self.control_types = {"Dial", "Delay", "Toggle", "Slider", "Tumbler", "Radio", "Switch"}
        self.control = {}
        self.signal = {}
        self.ad={}
        self.ad["control"] = {"power": "off", "system": "off", "autonomy": "disabled", "doors": "locked", "gear": "in park"}
        self.ad["operation"] = {"power": "okay", "compute": "okay", "vehicle": "okay", "sensors": "okay", "tires": "okay", "network": "okay"}
        self.ad_status_announce = True
        self.rclpy_init()
        self.ros = RosBridge(self)
        self.rclpy_activate()
        self.keep_alive_activate()

    def shutdown(self):
        self.log.info("skill.shutdown")
        self.cancel_all_repeating_events()
        self.rclpy_shutdown()

    def keep_active(self):
        self.log.info('skill.keep_active:')
        self.make_active()

    def converse(self, message=None):
        if message and self.ask_converse:
            self.log.info('skill.converse: %s' % message.data)
            if self.ask:
                if not message.data["utterances"]:
                    response = None
                    self.speak_dialog("none")
                else:
                    response = message.data["utterances"][0]
                    if response in self.ask_cancel or response in self.ask["options"]:
                        self.log.info('skill.converse: %s:%s' % (self.ask["signal"], response))
                        response_idx = -1
                        if response in self.ask_cancel:
                            response = "cancel"
                            response_idx = 0
                        else:
                            response_idx = self.ask["options"].index(response)
                        #self.ros.send_cmd_data({"cancel": self.ask["signal"]})
                        self.log.info('skill.converse: confirm? %s:%s' % (response, self.ask["confirm"]))
                        if "confirm" in self.ask:
                            self.log.info('skill.converse: confirm %s:%s' % (response, self.ask["confirm"]))
                            self.speak("%s." % response)
                            self.speak_dialog(self.ask["confirm"])
                        if response_idx >= 0:
                            self.ros.send_ctrl_data({self.ask["signal"]: int(self.ask["values"][response_idx])})
                        self.ask_converse = False
                        return True
                    else:
                        self.speak("%s, is not an option." % response)
                if self.ask["retries"] > 0:
                    self.ask["retries"] -= 1
                    if "speak" in self.ask:
                        self.speak(self.ask["dialog"], expect_response=True)
                    elif "dialog" in self.ask:
                        self.speak_dialog(self.ask["dialog"], expect_response=True)
                else:
                    self.log.info('skill.converse: out of retries')
                    #self.ros.send_cmd_data({"cancel": self.ask["signal"]})
                    #response = "#cancel"
                    #self.ros.send_ctrl_data({self.ask["signal"]: response})
                    self.ask_converse = False
                if self.ask_converse:
                    self.make_active()
                return True
            else:
                self.log.info('skill.converse: no question (%d)' % self.ask_converse)
                if self.ask_converse:
                    self.log.info('skill.converse: already cancelled')
                    self.ask_converse = False
                    return True
        return False

    def control_add(self, v):
        self.log.info('skill.control_add: "%s"' % v)
        #self.control[v["title"]] = v
        signal = v["signal"]
        title = v["title"]
        self.signal[signal] = v
        if "value" not in self.signal[signal]:
            self.signal[signal]["value"] = 0

    def change_value(self, k, v):
        o = self.signal[k]["value"]
        self.signal[k]["value"] = v
        if o == v:
            return
        t = self.signal[k]["type"]
        self.log.info('skill.change_value: %s=%d (%d)' % (k, v, o))
        vs = ""
        #if t in {"Dial", "Slider", "Gauge", "Tacho"}:
            #vs = String(v)
        if t in {"Delay", "Toggle", "Tumbler", "Radio"}:
            if "spoken" in self.signal[k]:
                vs = self.signal[k]["spoken"].split("|")[v].capitalize()
            else:
                vs = self.signal[k]["options"].split("|")[v].capitalize()
        elif t in {"Switch", "Led"}:
            vs = str(v)
            return
        else:
            vs = str(v)
        self.speak('%s %s.' % (self.signal[k]["title"], vs))

    def voice_ask_cancel(self, v):
        self.log.info('skill.voice_ask_cancel: "%s"' % v)
        if "signal" in self.ask:
            if self.ask["signal"] == v:
                self.ask = {}

    def voice_ask_options(self, v):
        self.log.info('skill.voice_ask_options: %s' % v)
        self.ask = v
        self.ask["response"] = None
        if not "retries" in self.ask:
            self.ask["retries"] = 3
        if not "signal" in self.ask:
            return False
        if not "data" in self.ask:
            self.ask["data"] = None
        if not "confirm" in self.ask:
            self.ask["confirm"] = None
        if "options" in self.ask:
            self.ask["options"] = v["options"].lower().split("|")
        else:
            return False
        if "values" in self.ask:
            self.ask["values"] = v["values"].lower().split("|")
        else:
            return False
        self.ask_converse = True
        if "speak" in self.ask:
            self.speak(self.ask["dialog"], expect_response=True)
        elif "dialog" in self.ask:
            self.speak_dialog(self.ask["dialog"], expect_response=True)
        else:
            self.ask_converse = False
        if self.ask_converse:
            self.make_active()
        return self.ask_converse

    @intent_file_handler('mkz.intent')
    def handle_demo_urban_mkz(self, message):
        self.cancel_all_repeating_events()
        if self.greeting:
            self.speak_dialog("greeting")
            self.greeting = False
        else:
            self.speak_dialog("reinit")
        self.rclpy_activate()
        self.keep_alive_activate()

    #@intent_file_handler('status.ad.mkz.intent')
    #def handle_ad_status_mkz(self, message):
        #s=message.data["utterance"][10:]
        #i1=s.index(" ")
        #i2=s[i1+1:].index(" ")
        #ad_type = s[0:i1]
        #ad_item = s[i1+1:i1+i2+1]
        #ad_value = s[i1+i2+2:]
        #self.log.info("ad status: type="+ad_type+" item="+ad_item+" value="+ad_value)
        #if (ad_type not in self.ad.keys()):
            #self.ad[ad_type]={}
        #else:
            #self.ad[ad_type][ad_item]=ad_value
        #if (self.ad_status_announce):
            #self.speak(ad_type+" status."+" the "+ad_item+" is "+ad_value, wait=True)

    @intent_file_handler('hmi.show.intent')
    def handle_show_hmi(self, message):
        sticky = False
        self.ui["uiIdx"] = 0
        self.ui["uiIdx_Sticky"] = 0
        for k in message.data["hmi"].split():
            self.log.info('handle_show_hmi: %s' % k)
            if k in self.uiIdxKeys:
                if self.ui["uiIdx"] > 0:
                    self.ui["uiIdx_Sticky"] = self.ui["uiIdx"]
                self.ui["uiIdx"] |= self.uiIdxKeys[k]
        data = {"uiIdx":self.ui["uiIdx"], "uiIdx_Sticky":self.ui["uiIdx_Sticky"]}
        self.log.info('skill.handle_show_hmi: %s' % data)
        self.ros.send_hmi_data(data)
        self.speak_dialog('confirm', wait=False)

    #@intent_file_handler('status.query.mkz.intent')
    #def handle_query_status_mkz(self, message):
        #self.gui["uiIdx"] = 2
        #s=message.data["utterance"][10:]
        #ad_type = message.data.get('type')
        #self.log.info("query status: type="+ad_type)
        #if (ad_type not in self.ad.keys()):
            #self.speak("there is no such status to report.", wait=True)
        #elif (len(self.ad[ad_type])==0):
            #if (ad_type[-1]=="s"):
                #self.speak("there are no "+ad_type+" to report.", wait=True)
            #else:
                #self.speak("there is no "+ad_type+" to report.", wait=True)
        #else:
            #self.speak("here is the "+ad_type+" status report.", wait=True)
            #idx=0
            #for ad_item, ad_value in self.ad[ad_type].items():
                #self.gui["controlIdx"] = idx
                #idx=idx+1
                #if (ad_item[-1]=="s"):
                    #self.speak("the "+ad_item+" are "+ad_value+".", wait=True)
                #else:
                    #self.speak("the "+ad_item+" is "+ad_value+".", wait=True)

def create_skill():
    return Mkz()
