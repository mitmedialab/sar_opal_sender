#!/usr/bin/env python
"""
Jacqueline Kory Westlund
May 2016

The MIT License (MIT)

Copyright (c) 2016 Personal Robots Group

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import argparse
import json
import rospy
from sar_opal_msgs.msg import OpalCommand  # For building opal messages.
from std_msgs.msg import Header  # Standard ROS msg header.


def opal_sender():
    """ Opal sender uses ROS to send messages via a rosbridge_server websocket
    connection to a SAR Opal tablet.
    """
    # Parse python arguments.
    parser = argparse.ArgumentParser(
            formatter_class=argparse.RawDescriptionHelpFormatter,
            description="Send a message to a"
            + " SAR Opal project tablet. Must have roscore and "
            + "rosbridge_server running for message to be sent.")
    parser.add_argument("-l", "--load", dest="loadme", action="append",
                        nargs="?", help="load the game object specified in "
                        "this json config file on the tablet")
    parser.add_argument("-t", "--touch", type=str, dest="touch",
                        choices=["enable", "e", "disable", "d"],
                        help="enable/disable touch events on tablet")
    parser.add_argument("-r", "--reset", action="store_true",
                        help="reload all objects and reset scene on tablet")
    parser.add_argument("-d", "--sidekick_do", dest="sidekick_do",
                        action="append", nargs="?", type=str,
                        help="tells sidekick to do specified action")
    parser.add_argument("-s", "--sidekick_say", dest="sidekick_say",
                        action="append", nargs="?", type=str,
                        help="tells sidekick to say specified speech")
    parser.add_argument("-c", "--clear", dest="clear_me", action="append",
                        nargs="?", type=str, default=None,
                        help="clear objects from tablet screen")
    parser.add_argument("-m", "--move", dest="moveme", action="append",
                        nargs="?", help="move the game object specified in "
                        "this json config file to the specified position on "
                        "the tablet")
    parser.add_argument("-i", "--highlight", dest="highlight",
                        action="append", nargs="?", type=str, help="highlight "
                        + "the specified game object")
    parser.add_argument("-k", "--keyframe", action="store_true",
                        help="request the state of all objects on the tablet")
    parser.add_argument("-f", "--fade", choices=["fade", "f", "unfade", "u"],
                        type=str, dest="fade", help="fade/unfade screen on "
                        "tablet")
    parser.add_argument("-q", "--quit", action="store_true",
                        help="quit the tablet app")
    parser.add_argument("-e", "--set_correct", dest="set_correct",
                        action="append", nargs="?", help="tag game objects as "
                        "correct or as incorrect")
    parser.add_argument("-w", "--correct", choices=["show", "s", "hide", "h"],
                        type=str, dest="correct", help="show/hide visual "
                        "feedback for correct/incorrect game objects")
    parser.add_argument("-u", "--setup_scene", dest="setup_scene",
                        action="append", nargs="?", help="set up initial game "
                        "scene for a social stories game")
    parser.add_argument("-o", "--story-selection", dest="story_selection",
                        action="append", nargs="?", type=str,
                        help="Load a storybook.")
    parser.add_argument("-b", "--buttons", type=str, dest="buttons",
                        choices=["show", "s", "hide", "h"],
                        help="Show/hide the flip buttons for a storybook.")
    parser.add_argument("-p", "--story-go-to-page", dest="story_page",
                        action="append", nargs="?", type=int,
                        help="Go to a page in a storybook.")

    args = parser.parse_args()
    print args

    # Now build a message based on the command:
    # Open ros up here, then run through the below and send all.

    # Start ROS node.
    pub = rospy.Publisher("/opal_tablet_command", OpalCommand, queue_size=10)
    rospy.init_node("opal_sender", anonymous=True)
    rnode = rospy.Rate(10)
    # Sleep to wait for subscribers.
    rnode.sleep()

    # Start building message.
    msg = OpalCommand()
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    # Fill in command and properties:

    # Load an object?
    if args.loadme:
        # For each object to load, send a message.
        for obj in args.loadme:
            # Parse config file to get details of object or background to load.
            try:
                with open(obj) as json_file:
                    json_data = json.load(json_file)
                print json_data
                msg.command = OpalCommand.LOAD_OBJECT
                # Add the object properties to the message (the json data).
                msg.properties = json.dumps(json_data)
            except ValueError as err:
                print "Error! Could not open or parse json config file!" \
                    "\n  Did you put only one game object config in" \
                    " the file?\n  Did you use valid json?" \
                    "\nError: {}".format(err)
            except IOError as err:
                print "Error! Couldn't open or couldn't parse json config!" \
                    "\n  Does the file exist in this directory, or did" \
                    " you specify the full file path?" \
                    "\n  Did you include the file extension, if there is" \
                    " one?\nError:{}".format(err)

    # Sidekick do an action.
    if args.sidekick_do:
        msg.command = OpalCommand.SIDEKICK_DO
        msg.properties = args.sidekick_do[0]

    # Sidekick say something.
    if args.sidekick_say:
        msg.command = OpalCommand.SIDEKICK_SAY
        msg.properties = args.sidekick_say[0]

    # Send enable or disable touch events command.
    if args.touch:
        msg.command = OpalCommand.ENABLE_TOUCH if args.touch == "enabled" \
                or args.touch == "e" else OpalCommand.DISABLE_TOUCH

    # Send reset scene and reload objects command.
    if args.reset:
        print "reload"
        msg.command = OpalCommand.RESET

    # Send clear scene command.
    if args.clear_me:
        print "clear"
        msg.command = OpalCommand.CLEAR
        if args.clear_me[0]:
            msg.properties = args.clear_me[0]

    # Send quit command.
    if args.quit:
        print "quit"
        msg.command = OpalCommand.EXIT

    # Send move object command: for each object to move, send a message.
    if args.moveme:
        for obj in args.moveme:
            # Parse config file to get details of object or background to load.
            try:
                with open(obj) as json_file:
                    json_data = json.load(json_file)
                print json_data
                msg.command = OpalCommand.MOVE_OBJECT
                # Add the object properties to the message (the json data).
                msg.properties = json.dumps(json_data)
            except ValueError as err:
                print "Error! Could not open or parse json config file!" \
                    "\n  Did you put only one game object config in" \
                    " the file?\n  Did you use valid json?" \
                    "\nError: {}".format(err)
            except IOError as err:
                print "Error! Couldn't open or couldn't parse json config!" \
                    "\n  Does the file exist in this directory, or did" \
                    " you specify the full file path?" \
                    "\n  Did you include the file extension, if there is" \
                    " one?\nError:{}".format(err)

    # Send highlight object command.
    if args.highlight:
        msg.command = OpalCommand.HIGHLIGHT_OBJECT
        msg.properties = args.highlight[0]

    # Send request keyframe command.
    if args.keyframe:
        print "request keyframe"
        msg.command = OpalCommand.REQUEST_KEYFRAME

    # Send fade or unfade screen command.
    if args.fade:
        msg.command = OpalCommand.FADE_SCREEN if args.touch == "fade" \
                or args.touch == "f" else OpalCommand.UNFADE_SCREEN

    # Send set correct command.
    if args.set_correct:
        for obj in args.set_correct:
            # Parse config file to get details of object or background to load.
            try:
                with open(obj) as json_file:
                    json_data = json.load(json_file)
                print json_data
                msg.command = OpalCommand.SET_CORRECT
                # Add the object properties to the message (the json data).
                msg.properties = json.dumps(json_data)
            except ValueError as err:
                print "Error! Could not open or parse json config file!" \
                    "\n  Did you put only one game object config in" \
                    " the file?\n  Did you use valid json?" \
                    "\nError: {}".format(err)
            except IOError as err:
                print "Error! Couldn't open or couldn't parse json config!" \
                    "\n  Does the file exist in this directory, or did" \
                    " you specify the full file path?" \
                    "\n  Did you include the file extension, if there is" \
                    " one?\nError:{}".format(err)

    # Send show correct or hide correct command.
    if args.correct:
        msg.command = OpalCommand.SHOW_CORRECT if args.correct == "show" \
                or args.correct == "s" else OpalCommand.HIDE_CORRECT

    # Send setup social story scene message.
    if args.setup_scene:
        # For each scene to setup, send a message.
        # It would be weird to do setup more than once, since the scene
        # clears before setup (so only the last setup would really count).
        for obj in args.setup_scene:
            # Parse config file to get details of scene.
            try:
                with open(obj) as json_file:
                    json_data = json.load(json_file)
                print json_data
                msg.command = OpalCommand.SETUP_STORY_SCENE
                # Add the object properties to the message (the json data).
                msg.properties = json.dumps(json_data)
            except ValueError as err:
                print "Error! Could not open or parse json config file!" \
                    "\n  Did you put only one game object config in" \
                    " the file?\n  Did you use valid json?" \
                    "\nError: {}".format(err)
            except IOError as err:
                print "Error! Couldn't open or couldn't parse json config!" \
                    "\n  Does the file exist in this directory, or did" \
                    " you specify the full file path?" \
                    "\n  Did you include the file extension, if there is" \
                    " one?\nError:{}".format(err)

    if args.story_selection:
        print "Selecting story: {}".format(args.story_selection[0])
        msg.command = OpalCommand.STORY_SELECTION
        msg.properties = args.story_selection[0]

    if args.buttons:
        msg.command = OpalCommand.STORY_SHOW_BUTTONS if args.buttons == "s" \
            or args.buttons == "show" else OpalCommand.STORY_HIDE_BUTTONS

    if args.story_page:
        msg.command = OpalCommand.STORY_GO_TO_PAGE
        msg.properties = str(args.story_page[0])

    # Send Opal message to tablet game.
    pub.publish(msg)
    rospy.loginfo(msg)
    rnode.sleep()


if __name__ == "__main__":
    try:
        opal_sender()
    except rospy.ROSInterruptException:
        print "ROSnode shutdown"
