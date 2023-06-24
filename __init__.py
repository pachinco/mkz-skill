
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
        self.sub_hmi = self.create_subscription(String, 'cmd', self.sub_cmd_rcv, 10)
        self.sub_hmi  # prevent unused variable warning
        self.pub_ctrl = self.create_publisher(String, 'ctrl', 10)
        self.pub_hmi = self.create_publisher(String, 'hmi', 10)

    def sub_cmd_rcv(self, msg):
        self.log.info('sub_cmd_rcv: "%s"' % msg.data)
        c = json.loads(msg.data.replace("'", '"'))
        for k,v in c.items():
            self.log.info('pub_hmi_snd: %s:%s', k, v)
           

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
        msg.data = message.dumps(message)
        self.log.info('ros_ctrl_send: %s', msg.data)
        self.ros.pub_ctrl_snd(msg)
        rclpy.spin_once(self.ros, timeout_sec=0)

    def ros_hmi_send(self, message):
        msg = String()
        msg.data = message.dumps(message)
        self.log.info('ros_hmi_send: %s', msg.data)
        self.ros.pub_hmi_snd(msg)
        rclpy.spin_once(self.ros, timeout_sec=0)

    def ros_init(self):
        #print("MycroftSkill: ros_init");
        self.log.info("Mkz: ros_init");
        self._args=sys.argv
        self.log.info(self._args)
        self._context = rclpy.context.Context()
        self.log.info(self._context)
        #rclpy.init(args=self._args, context=self._context)
        if not rclpy.utilities.ok():
            rclpy.init(args=self._args)
        self.ros = RosBridge(self)
        self.gui.register_handler('send.ctrl.event', self.ros_ctrl_send)
        self.gui.register_handler('send.hmi.event', self.ros_hmi_send)

    def ros_activate(self):
        self.schedule_repeating_event(self.ros_spin_once, None, 0.2)

    def ros_spin_once(self):
        rclpy.spin_once(self.ros, timeout_sec=0)
        
    #@resting_screen_handler('MKZ homescreen')
    #def handle_homescreen(self, message):
        #self.gui.clear()
        #self.enclosure.display_manager.remove_active()
        #self.log.info('Activating MKZ homescreen')
        #self.gui.show_image("image/mkz_background_stage_day.png", override_idle=True, override_animations=True)

    @intent_file_handler('mkz.intent')
    def handle_demo_urban_mkz(self, message):
        self.cancel_all_repeating_events()
        #self.ros_init()
        #self.gui.clear()
        #self.gui.register_handler('mkz-urban-demo-skill.route_new', self._route_new)
        #self.gui.register_handler('mkz-urban-demo-skill.route_position', self._route_position)
        #self.gui.register_handler('mkz-urban-demo-skill.route_next_segment', self._route_next_instruction)
        #self.gui["uiIdx"] = -2
        #self.gui["routeReady"] = False
        #self.gui["routeTotalTime"] = 0
        #self.gui["routeTotalDistance"] = 0
        #self.gui["routeNum"] = 0
        #self.gui["routePath"] = 0
        #self.gui["controlIdx"] = 0
        #self.gui["routeSegments"] = 0
        #self.gui["routeSegment"] = 0
        #self.gui["routeSegmentNext"] = False
        #self.gui["routeModel"] = []
        #self.gui["routeDistance"] = 0
        #self.gui["routePositionLat"] = 0
        #self.gui["routePositionLon"] = 0
        #self.gui["routeDirection"] = 0
        #self.gui["routeTime"] = 0
        #self.gui["routeInstruction"] = ""
        #self.gui["routeTimeToNext"] = 0
        #self.gui["routeDistanceToNext"] = 0
        #self.gui["routeNext"] = False
        #self.gui["routeNextAnnouced"] = False
        #self.gui["routeNextSegment"] = 0;
        #self.gui["routeNextDistance"] = 0
        #self.gui["routeNextPositionLat"] = 0
        #self.gui["routeNextPositionLon"] = 0
        #self.gui["routeNextDirection"] = 0
        #self.gui["routeNextTime"] = ""
        #self.gui["routeNextInstruction"] = ""
        #self.gui["modeAutonomous"] = False
        #self.gui["modeRoute"] = False
        #self.gui["modeMarker"] = True
        #self.gui["modeFollow"] = True
        #self.gui["modeNorth"] = False
        #self.gui["mode3D"] = True
        #self.gui["modeTraffic"] = False
        #self.gui["modeNight"] = False
        #self.gui["carPositionLat"] = 0
        #self.gui["carPositionLon"] = 0
        #self.gui["carPosition"] = {"lat": 37.3964, "lon": -122.034}
        #self.gui["uiButtons"] = [{"ui": "map", "idx": 1, "image": "../images/map-road.png"},
                                  #{"ui": "car", "idx": 2, "image": "../images/mode-car.png"},
                                  #{"ui": "config", "idx": 0, "image": "../images/sliders-solid.png"},
                                  #{"ui": "music", "idx": 3, "image": "../images/music-solid.png"},
                                  #{"ui": "weather", "idx": 4, "image": "../images/cloud-sun-rain-solid.png"}]
        #self.gui["actionsList"] = []
        #self.gui["statusList"] = [{"text": "▾ Vehicle ❌"},
                                  #{"text": "  ▾ Doors ❌"},
                                  #{"text": "      Front Left ❌"},
                                  #{"text": "      Front Right ✓"},
                                  #{"text": "      Rear Left ✓"},
                                  #{"text": "      Rear Right ✓"},
                                    #{"text": "▸ Sensors ✓"},
                                    #{"text": "▸ Driver ✓"},
                                    #{"text": "▸ Compute ✓"},
                                    #{"text": "▸ Communication ✓"},
                                    #{"text": "▸ Environment ✓"}]
        #self.enclosure.display_manager.remove_active()
        #play_proc = play_wav(str(self.sound_file_path))
        #self.gui.show_page(str(self.mkz_home_ui), override_idle=True)
        self.speak_dialog('mkz', wait=True)
        #self.schedule_repeating_event(self._update_display_time, None, 10)
        #self.schedule_event(self._whats_next, 3)
        self.ros_activate()
        #self.cluster = Cluster()
        #self.cluster.start()

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
        #self.gui.clear()
        #self.enclosure.display_manager.remove_active()
        #self.speak("What's next?", expect_response=True, wait=True)
        #self.gui.show_page(str(self.mkz_list_ui))
        #self.gui["uiIdx"] = -1
        #self.schedule_event(self._switch_config, 3)

    #def _switch_config(self):
        #self.gui["actionsList"] = []
        #self.gui["actionsList"] = [{"text": "Power", "status": "off", "image": "../images/Power-button.png"},
                                    #{"text": "System", "status": "off", "image": "../images/Start-button.png"},
                                    #{"text": "Autonomy", "status": "disabled", "image": "../images/Settings-symbol.png"},
                                    #{"text": "Doors", "status": "locked", "image": "../images/Settings-symbol.png"},
                                    #{"text": "Gear", "status": "park", "image": "../images/Settings-symbol.png"}]
        #self.gui["uiIdx"] = 2
        #self.schedule_event(self._add_config, 1)
        
    #def _add_config(self):
        #self.gui["uiIdx"] = 2
        #self.schedule_event(self._back_map, 5)

    #def _back_map(self):
        #self.gui["uiIdx"] = 1
        #self.schedule_event(self._back_config, 5)

    #def _back_config(self):
        #self.gui["uiIdx"] = 0
        #self.schedule_event(self._back_home, 5)

    #def _back_home(self):
        #self.gui["uiIdx"] = -1

    #def _blink(self):
        #self.gui["brakeOn"] = not self.gui["brakeOn"]
        #self.gui["blinkLeft"] = not self.gui["blinkLeft"]
        #self.gui["blinkRight"] = not self.gui["blinkRight"]

    #def _update_display_time(self):
        #dt = datetime.now()
        #dt_str = dt.strftime("%I:%M%p %a %b %-d")
        #dt_str = dt_str.replace("AM","am")
        #dt_str = dt_str.replace("PM","pm")
        #self.log.info("datetime: "+dt_str)
        #hh_mm = nice_time(dt, speech=False, use_24hour=False)
        #self.gui["datetime"] = dt_str

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
