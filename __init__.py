
import rclpy
import sys
import ast
import json
#import threading

from time import strftime, localtime
from rclpy.node import Node
from std_msgs.msg import String

from mycroft import MycroftSkill, intent_file_handler
from pathlib import Path
from mycroft.util import play_wav
from mycroft.skills import resting_screen_handler
from datetime import datetime

class RosBridge(Node):
    def __init__(self, skill):
        super().__init__('ros')
        #self.gui = skill.gui
        self.log = skill.log
        self.skill = skill
        self.sub_hmi = self.create_subscription(String, 'cmd', self.sub_cmd_rcv, 10)
        self.sub_hmi  # prevent unused variable warning
        self.pub_ctrl = self.create_publisher(String, 'ctrl', 10)
        self.pub_hmi = self.create_publisher(String, 'hmi', 10)

    def sub_cmd_rcv(self, msg):
        self.log.info('sub_cmd_rcv: "%s"' % msg.data)
        c = json.loads(msg.data.replace("'", '"'))
        for k,v in c.items():
            self.log.info('pub_hmi_snd: %s:%s' % (k, v))
            if k == "speak":
                self.skill.speak(v, wait=True)
            elif k == "dialog":
                self.skill.speak_dialog(v, wait=True)

    def pub_ctrl_snd(self, msg):
        self.log.info('pub_ctrl_snd: %s', msg.data)
        self.pub_ctrl.publish(msg)

    def pub_hmi_snd(self, msg):
        self.log.info('pub_hmi_snd: %s', msg.data)
        self.pub_hmi.publish(msg)

class Mkz(MycroftSkill):
    def __init__(self):
        MycroftSkill.__init__(self)
        self.sound_file_path = Path(__file__).parent.joinpath("sounds", "mkz-welcome-chime2.wav")

    def initialize(self):
        self.uiIdx = {"none": 0, "map": 2, "addresses": 4, "rolodex": 4, "locations": 4, "status": 8, "diagnostics": 8, "control": 16, "media": 32, "music": 32, "weather": 64, "news": 128}
        self.ad={}
        self.ad["control"] = {"power": "off", "system": "off", "autonomy": "disabled", "doors": "locked", "gear": "in park"}
        self.ad["operation"] = {"power": "okay", "compute": "okay", "vehicle": "okay", "sensors": "okay", "tires": "okay", "network": "okay"}
        self.ad_status_announce = True
        self.ros_init()

    def shutdown(self):
        self.log.info("Mkz: shutdown")
        self.cancel_all_repeating_events()
        if self.ros is not None:
            self.ros.destroy_node()
        if rclpy.utilities.ok():
            rclpy.shutdown()

    def ros_ctrl_send(self, message):
        msg = String()
        msg.data = json.dumps(message, separators=(',', ':'))
        msg.data = msg.data.replace("'", '"')
        self.log.info('ros_ctrl_send: %s', msg.data)
        self.ros.pub_ctrl_snd(msg)
        rclpy.spin_once(self.ros, timeout_sec=0)

    def ros_hmi_send(self, message):
        msg = String()
        msg.data = json.dumps(message, separators=(',', ':'))
        msg.data = msg.data.replace("'", '"')
        self.log.info('ros_hmi_send: %s', msg.data)
        self.ros.pub_hmi_snd(msg)
        rclpy.spin_once(self.ros, timeout_sec=0)

    def ros_init(self):
        self.log.info("Mkz: ros_init");
        self._args=sys.argv
        self.log.info(self._args)
        self._context = rclpy.context.Context()
        self.log.info(self._context)
        #rclpy.init(args=self._args, context=self._context)
        if not rclpy.utilities.ok():
            rclpy.init(args=self._args)
        self.ros = RosBridge(self)

    def ros_activate(self):
        self.schedule_repeating_event(self.ros_spin_once, None, 0.1)

    def ros_spin_once(self):
        rclpy.spin_once(self.ros, timeout_sec=0)

    @intent_file_handler('mkz.intent')
    def handle_demo_urban_mkz(self, message):
        self.cancel_all_repeating_events()
        self.speak_dialog('mkz', wait=True)
        self.ros_activate()

    @intent_file_handler('status.ad.mkz.intent')
    def handle_ad_status_mkz(self, message):
        s=message.data["utterance"][10:]
        i1=s.index(" ")
        i2=s[i1+1:].index(" ")
        ad_type = s[0:i1]
        ad_item = s[i1+1:i1+i2+1]
        ad_value = s[i1+i2+2:]
        self.log.info("ad status: type="+ad_type+" item="+ad_item+" value="+ad_value)
        if (ad_type not in self.ad.keys()):
            self.ad[ad_type]={}
        else:
            self.ad[ad_type][ad_item]=ad_value
        if (self.ad_status_announce):
            self.speak(ad_type+" status."+" the "+ad_item+" is "+ad_value, wait=True)

    @intent_file_handler('hmi.show.intent')
    def handle_show_hmi(self, message):
        uiIdx = 0
        for k in message.data["hmi"].split():
            self.log.info('handle_show_hmi: %s' % k)
            if k in self.uiIdx:
                uiIdx |= self.uiIdx[k]
        self.log.info('handle_show_hmi: uiIdx=%d' % uiIdx)
        d = String()
        d.data = '{"uiIdx":%d}' % uiIdx
        self.ros.pub_hmi_snd(d)

    @intent_file_handler('status.query.mkz.intent')
    def handle_query_status_mkz(self, message):
        #self.gui.clear()
        #self.enclosure.display_manager.remove_active()
        self.gui["uiIdx"] = 2
        s=message.data["utterance"][10:]
        ad_type = message.data.get('type')
        self.log.info("query status: type="+ad_type)
        if (ad_type not in self.ad.keys()):
            self.speak("there is no such status to report.", wait=True)
        elif (len(self.ad[ad_type])==0):
            if (ad_type[-1]=="s"):
                self.speak("there are no "+ad_type+" to report.", wait=True)
            else:
                self.speak("there is no "+ad_type+" to report.", wait=True)
        else:
            self.speak("here is the "+ad_type+" status report.", wait=True)
            idx=0
            for ad_item, ad_value in self.ad[ad_type].items():
                self.gui["controlIdx"] = idx
                idx=idx+1
                if (ad_item[-1]=="s"):
                    self.speak("the "+ad_item+" are "+ad_value+".", wait=True)
                else:
                    self.speak("the "+ad_item+" is "+ad_value+".", wait=True)

    #def _whats_next(self):
        #self.speak("What's next?", expect_response=True, wait=True)
        #self.schedule_event(self._switch_config, 3)

    def _route_new(self, message):
        #self.route_path = 0
        if (len(message.data["string"])>0):
            self.speak(message.data["string"], wait=True)
        #self.speak(self.gui["routeInstruction"])
        self.log.info("total time: %d seconds / %d meters",self.gui["routeTime"],self.gui["routeDistance"])
        self.log.info("position: %f,%f -> %f,%f",self.gui["routePositionLat"],self.gui["routePositionLon"],self.gui["routeNextPositionLat"],self.gui["routeNextPositionLon"])
        self.log.info("segments: %d/%d",self.gui["routeSegment"],self.gui["routeSegments"])
        if (self.gui["routeNext"]):
            self.log.info("next: %d seconds",self.gui["routeTimeToNext"])
        #self.path = ast.literal_eval(self.gui["routePath"])
        #self.log.info("path: %d",len(self.path))
        #self.log.info(self.path[self.route_path])
        #self.gui["carPosition"] = {"lat": self.path[self.route_path]["lat"], "lon": self.path[self.route_path]["lon"]}
        if (self.gui["routeNext"]):
            self.schedule_event(self._route_next_instruction, min(self.gui["routeTimeToNext"]/3,1))

    def _route_next_instruction(self):
        self.log.info("routeDistanceToNext: %f",self.gui["routeDistanceToNext"])
        self.log.info("routeNextDirection: %d",self.gui["routeNextDirection"])
        self.log.info("routeNextDirection: %d",self.gui["routeNextDirection"])
        if (self.gui["modeAutonomous"]):
            self.speak("Next, "+self.gui["routeNextInstruction"])
        else:
            if (round(self.gui["routeDistanceToNext"])>0):
                self.speak("In "+str(round(self.gui["routeDistanceToNext"]))+" meters. "+self.gui["routeNextInstruction"])
        #self.schedule_event(self._route_wait_next_position, 1)

    def _route_position(self, message):
        lat = message.data["lat"]
        lon = message.data["lon"]
        #self.log.info("route position: %f %f",lat,lon)
        self.log.info("route position: %f,%f -> %f,%f",lat,lon,self.gui["routeNextPositionLat"],self.gui["routeNextPositionLon"])
        self.log.info("segment: %d(%d) / path: %d",message.data["segment"],self.gui["routeSegments"],message.data["path"])
        if (self.gui["routeNext"]\
            and not self.gui["routeNextAnnouced"]\
            and (abs(self.gui["routeNextPositionLat"]-lat)<0.0001)\
            and (abs(self.gui["routeNextPositionLon"]-lon)<0.0001)):
            self.gui["routeNextAnnouced"] = True
            self.speak(self.gui["routeNextInstruction"], wait=True)
            self.gui["routeSegmentNext"] = not self.gui["routeSegmentNext"]
        #else:
            #self.schedule_event(self._route_wait_next_position, 1)

    #def _route_next_path(self):
        #self.route_path = self.route_path+1
        #self.log.info("route_path=%d",self.route_path)
        #if (self.route_path<len(self.path)):
            #self.log.info(self.path[self.route_path])
            #self.gui["carPosition"] = {"lat": self.path[self.route_path]["lat"], "lon": self.path[self.route_path]["lon"]}
            #self.schedule_event(self._route_next_path, 1)
        #else:
            #route_segment = self.gui["routeSegment"]+1
            #if (route_segment<self.gui["routeSegments"]):
                #self.gui["routeSegment"] = route_segment
            #if (self.gui["routeNext"]):
                #self.schedule_event(self._route_next_segment, 2)
            
    #def _route_next_segment(self):
        #self.gui["carPosition"] = {"latitude": self.gui["routeNextPositionLat"], "longitude": self.gui["routeNextPositionLon"]}
        #if (self.gui["modeAutonomous"]):
            #self.speak("Next, "+self.gui["routeNextInstruction"], wait=True)
        #else:
            #self.speak("In "+str(round(self.gui["routeDistanceToNext"]))+" meters. "+self.gui["routeNextInstruction"], wait=True)

def create_skill():
    return Mkz()
