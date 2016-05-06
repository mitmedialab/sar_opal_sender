# sar\_opal\_sender
Opal sender is a ROS package that allows you to enter commands to send to a SAR Opal tablet via a rosbridge\_server websocket connection.

Note that for communication with the tablet to occur, you need to have the rosbridge\_server running, using the following command:

roslaunch rosbridge\_server rosbridge\_websocket.launch

## Usage
opal\_sender.py \[-h\] \[-l \[LOADME\]\] \[-t \{enable,e,disable,d\}\] \[-r\] \[-d \[SIDEKICK\_DO\]\] \[-s \[SIDEKICK\_SAY\]\] \[-f \{fade,f,unfade,u\}\] \[-e \[SET\_CORRECT\]\] \[-w \{show,s,hide,h\}\]

Optional arguments:

-h, --help  
show this help message and exit

-l \[LOADME\], --load \[LOADME\]  
tell tablet to load the game object specified in the LOADME json config file

-t \{enable,e,disable,d\}, --touch \{enable,e,disable,d\}  
enable or disable touch events on the tablet

-r, --reset  
reload all objects and reset secene on the tablet

-d \[SIDEKICK\_DO\], --sidekick\_do \[SIDEKICK\_DO\]  
tells virtual sidekick on tablet to do the specified action

-s \[SIDEKICK\_SAY\], --sidekick\_say [SIDEKICK\_SAY]  
tells virtual sidekick on tablet to say the specified speech

-c, --clear
clear all objects from tablet scene

-m \[MOVEME\], --move \[MOVEME\]
move specified game object to specified position

-i \[OBJECT\], --highlight \[OBJECT\]
highlight specified game object

-k, --keyframe
request state of all game objects on tablet

-q, --quit
quit the tablet app

-f \{fade,f,unfade,u\}, --fade \{fade,f,unfade,u\}
fade or unfade the tablet screen

-e \[SET\_CORRECT\], --set\_correct \[SET\_CORRECT\]
tag a set of objects as correct or incorrect responses
 
-w \{show,s,hide,h\}, --correct \{show,s,hide,h\}
show or hide visual feedback for correct or incorrect responses

-c \[SET\_ME\_UP\], --setup\_scene \[SET\_ME\_UP\]
set up initial game scene for social stories game

## JSON config files

Several example config files are included here. Each file should contain one
valid JSON object with the relevant properties for the object to load. For
example, if you want to load the "dragon" image as a draggable PlayObject, with
the audio file "chimes" attached (so that "chimes" plays whenever you tap the
object), and you want this object to be created at /(0,0,0/) in the tablet
screen world with the image scaled by /(100,100,100/), you would write:

> {  "name": "dragon",  "tag": "PlayObject",  "draggable": "true",
> "audioFile": "chimes",  "position": [0,0,0], "scale": [100,100,100] }

Here is another example:
> {  
>    "name": "ball1",  
>    "tag": "PlayObject",  
>    "draggable": "false",  
>    "audioFile": "wakeup",  
>    "position": [300,10,0],  
>    "scale": [100,100,100]
> }

Note that you don't have to put the numbers in quotes - if you do, they'll be strings, if you don't, they'll be numbers; either way, the tablet will parse it correctly.

If you want to load a background image instead of a draggable object, you would specify just the name of the image and the tag:
> {  
>    "name": "playground",  
>    "tag": "Background"  
> }  

If you want to specify which object to move, you would write:
> {
>     "name": "dragon",
>     "destination": [100,200,0]
> }

If you want to tag objects as correct or incorrect, you would write:
> {
>       "correct":["dragon"],
>       "incorrect":["ball1","cat"]
> }

If you want to set up a social stories game scene, you specify whether the scenes will be in order or not, the number of scenes in thes tory, and the number of answer options:
> {
>   "numScenes":"4",
>   "scenesInOrder":true,
>   "numAnswers":"4"
> }

