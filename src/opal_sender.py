#!/usr/bin/env python

# Jacqueline Kory Westlund
# May 2016
#
# The MIT License (MIT)
#
# Copyright (c) 2016 Personal Robots Group
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
from sar_opal_msgs.msg import OpalCommand
import argparse
import json
from std_msgs.msg import Header # standard ROS msg header

# Opal sender uses ROS to send messages via a rosbridge_server 
# websocket connection to a SAR Opal tablet. 
def opal_sender():
    # parse python arguments 
    parser = argparse.ArgumentParser(
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description='Send a message to a' 
            + ' SAR Opal project tablet. Must have roscore and '
            + 'rosbridge_server running for message to be sent.')
    parser.add_argument('-l', '--load', dest='loadme', action='append', nargs='?',
            help='load the game object specified in this json config file' +
            ' on the tablet')
    parser.add_argument('-t', '--touch', choices=['enable','e','disable','d'],
            type=str, dest='touch',help='enable/disable touch events on tablet')
    parser.add_argument('-r', '--reset', action='store_true',
            help='reload all objects and reset scene on tablet')
    parser.add_argument('-d', '--sidekick_do', dest='sidekick_do', 
            action='append', nargs='?', type=str,
            help='tells sidekick to do specified action')
    parser.add_argument('-s', '--sidekick_say', dest='sidekick_say', 
            action='append', nargs='?', type=str,
            help='tells sidekick to say specified speech')
    parser.add_argument('-c', '--clear', action='store_true',
            help='clear all objects from tablet screen')
    parser.add_argument('-m', '--move', dest='moveme', action='append',
            nargs='?', help='move the game object specified in this json'
            +' config file to the specified position on the tablet')
    parser.add_argument('-i', '--highlight', dest='highlight',
            action='append', nargs='?', type=str, help='highlight '
            + 'the specified game object')
    parser.add_argument('-k', '--keyframe', action='store_true',
            help='request the state of all objects on the tablet')
    parser.add_argument('-f', '--fade', choices=['fade','f','unfade','u'],
            type=str, dest='fade', help='fade/unfade screen on tablet')
    parser.add_argument('-q', '--quit', action='store_true', 
            help='quit the tablet app')
    parser.add_argument('-e', '--set_correct', dest='set_correct',
            action='append', nargs='?', help='tag game objects as correct' +
            ' or as incorrect')
    parser.add_argument('-w', '--correct', choices=['show','s','hide','h'],
            type=str, dest='correct', help='show/hide visual feedback ' +
            'for correct/incorrect game objects')
    parser.add_argument('-u', '--setup_scene', dest='setup_scene',
            action='append', nargs='?', help='set up initial game scene for'
            + ' a social stories game')
    
    args = parser.parse_args()
    print(args)
    
    # now build a message based on the command:
    # open ros up here, then run through the below and send all

    # start ROS node
    pub = rospy.Publisher('opal_tablet_command', OpalCommand, queue_size=10)
    rospy.init_node('opal_sender', anonymous=True)
    r = rospy.Rate(10) # spin at 10 Hz
    r.sleep() # sleep to wait for subscribers


    # load an object
    if args.loadme:
        # for each object to load, send a message
        for obj in args.loadme:
            # parse config file to get details of object or
            # background to load
            try:
                with open (obj) as json_file:
                    json_data = json.load(json_file)
                print(json_data)
                 # build message
                msg = OpalCommand()
                # add header
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.command = OpalCommand.LOAD_OBJECT
                # add the object properties to the message 
                # (the loaded json data)
                msg.properties = json.dumps(json_data) 
                # send Opal message to tablet game
                pub.publish(msg)
                rospy.loginfo(msg)
                r.sleep()
            except ValueError as e:
                print('Error! Could not open or parse json config file!'
                    + '\n  Did you put only one game object config in'
                    + ' the file?\n  Did you use valid json?'
                    + '\nError: %s' % e)
            except IOError as e:
                print('Error! Could not open or could not parse json '
                       +'config file!'
                    + '\n  Does the file exist in this directory, or did'
                    + ' you specify the full file path?'
                    + '\n  Did you include the file extension, if there is'
                    + ' one?\nError: %s' % e)

    # sidekick do an action
    if args.sidekick_do:
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = OpalCommand.LOAD_OBJECT
        msg.command = OpalCommand.SIDEKICK_DO
        msg.properties = args.sidekick_do[0]
        # send Opal message to tablet game
        pub.publish(msg)
        rospy.loginfo(msg)
        r.sleep()

    # sidekick say something
    if args.sidekick_say:
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = OpalCommand.LOAD_OBJECT
        msg.command = OpalCommand.SIDEKICK_SAY
        msg.properties = args.sidekick_say[0]
        # send Opal message to tablet game
        pub.publish(msg)
        rospy.loginfo(msg)
        r.sleep()

    # send enable or disable touch events command
    if args.touch:
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = OpalCommand.LOAD_OBJECT
        msg.command = OpalCommand.ENABLE_TOUCH if args.touch == 'enabled' or args.touch == 'e' else OpalCommand.DISABLE_TOUCH
        # send Opal message to tablet game
        pub.publish(msg)
        rospy.loginfo(msg)
        r.sleep()

    # send reset scene and reload objects command
    if args.reset:
        print('reload');
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = OpalCommand.LOAD_OBJECT
        msg.command = OpalCommand.RESET
        # send Opal message to tablet game
        pub.publish(msg)
        rospy.loginfo(msg)
        r.sleep()

    # send clear scene command
    if args.clear:
        print('clear');
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = OpalCommand.LOAD_OBJECT
        msg.command = OpalCommand.CLEAR
        # send Opal message to tablet game
        pub.publish(msg)
        rospy.loginfo(msg)
        r.sleep()
    
    # send quit command
    if args.quit:
        print('quit');
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = OpalCommand.LOAD_OBJECT
        msg.command = OpalCommand.EXIT
        # send Opal message to tablet game
        pub.publish(msg)
        rospy.loginfo(msg)
        r.sleep()


    # send move object command
    # for each object to move, send a message
    if args.moveme:
        for obj in args.moveme:
            # parse config file to get details of object or
            # background to load
            try:
                with open (obj) as json_file:
                    json_data = json.load(json_file)
                print(json_data)
                 # build message
                msg = OpalCommand()
                # add header
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.command = OpalCommand.LOAD_OBJECT
                msg.command = OpalCommand.MOVE_OBJECT
                # add the object properties to the message 
                # (the loaded json data)
                msg.properties = json.dumps(json_data) 
                # send Opal message to tablet game
                pub.publish(msg)
                rospy.loginfo(msg)
                r.sleep()
            except ValueError as e:
                print('Error! Could not open or parse json config file!'
                    + '\n  Did you put only one game object config in'
                    + ' the file?\n  Did you use valid json?'
                    + '\nError: %s' % e)
            except IOError as e:
                print('Error! Could not open or could not parse json '
                       +'config file!'
                    + '\n  Does the file exist in this directory, or did'
                    + ' you specify the full file path?'
                    + '\n  Did you include the file extension, if there is'
                    + ' one?\nError: %s' % e)

    # send highlight object command
    if args.highlight:
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = OpalCommand.LOAD_OBJECT
        msg.command = OpalCommand.HIGHLIGHT_OBJECT
        msg.properties = args.highlight[0]
        # send Opal message to tablet game
        pub.publish(msg)
        rospy.loginfo(msg)
        r.sleep()

    # send request keyframe command
    if args.keyframe:
        print('request keyframe');
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = OpalCommand.LOAD_OBJECT
        msg.command = OpalCommand.REQUEST_KEYFRAME
        # send Opal message to tablet game
        pub.publish(msg)
        rospy.loginfo(msg)
        r.sleep()

    # send fade or unfade screen command
    if args.fade:
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = OpalCommand.LOAD_OBJECT
        msg.command = OpalCommand.FADE_SCREEN if args.touch == 'fade' or args.touch == 'f' else OpalCommand.UNFADE_SCREEN
        # send Opal message to tablet game
        pub.publish(msg)
        rospy.loginfo(msg)
        r.sleep()

    # send set correct command 
    if args.set_correct:
        for obj in args.set_correct:
            # parse config file to get details of object or
            # background to load
            try:
                with open (obj) as json_file:
                    json_data = json.load(json_file)
                print(json_data)
                # build message
                msg = OpalCommand()
                # add header
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.command = OpalCommand.LOAD_OBJECT
                msg.command = OpalCommand.SET_CORRECT
                # add the object properties to the message 
                # (the loaded json data)
                msg.properties = json.dumps(json_data) 
                # send Opal message to tablet game
                pub.publish(msg)
                rospy.loginfo(msg)
                r.sleep()
            except ValueError as e:
                print('Error! Could not open or parse json config file!'
                    + '\n Did you use valid json?\nError: %s' % e)
            except IOError as e:
                print('Error! Could not open or could not parse json '
                       +'config file!'
                    + '\n  Does the file exist in this directory, or did'
                    + ' you specify the full file path?'
                    + '\n  Did you include the file extension, if there is'
                    + ' one?\nError: %s' % e)

    # send show correct or hide correct command
    if args.correct:
        # build message
        msg = OpalCommand()
        # add header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.command = OpalCommand.LOAD_OBJECT
        msg.command = OpalCommand.SHOW_CORRECT if args.correct == 'show' or args.correct == 's' else OpalCommand.HIDE_CORRECT
        # send Opal message to tablet game
        pub.publish(msg)
        rospy.loginfo(msg)
        r.sleep()

    # send setup social story scene message 
    if args.setup_scene:
        # for each scene to setup, send a message
        # it would be weird to do setup more than once, since the scene
        # clears before setup (so only the last setup would really count)
        for obj in args.setup_scene:
            # parse config file to get details of scene 
            try:
                with open (obj) as json_file:
                    json_data = json.load(json_file)
                print(json_data)
                 # build message
                msg = OpalCommand()
                # add header
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.command = OpalCommand.LOAD_OBJECT
                msg.command = OpalCommand.SETUP_STORY_SCENE
                # add the object properties to the message 
                # (the loaded json data)
                msg.properties = json.dumps(json_data) 
                # send Opal message to tablet game
                pub.publish(msg)
                rospy.loginfo(msg)
                r.sleep()
            except ValueError as e:
                print('Error! Could not open or parse json config file!'
                    + '\n  Did you put only one scene setup config in'
                    + ' the file?\n  Did you use valid json?'
                    + '\nError: %s' % e)
            except IOError as e:
                print('Error! Could not open or could not parse json '
                       +'config file!'
                    + '\n  Does the file exist in this directory, or did'
                    + ' you specify the full file path?'
                    + '\n  Did you include the file extension, if there is'
                    + ' one?\nError: %s' % e)


        
if __name__ == '__main__':
    try:
        opal_sender()
    except rospy.ROSInterruptException:
        print('ROSnode shutdown')
