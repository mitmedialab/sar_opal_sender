# sar\_opal\_sender

Opal sender is a ROS package that allows you to enter commands to send to a SAR
Opal game via a rosbridge\_server websocket connection.

Note that for communication with an Opal game to occur, you need to have the
rosbridge\_server running, using the following command:

`roslaunch rosbridge\_server rosbridge\_websocket.launch`

## Usage

opal\_sender.py \[-h\] \[-l \[LOADME\]\] \[-t \{enable,e,disable,d\}\] \[-r\]
\[-d \[SIDEKICK\_DO\]\] \[-s \[SIDEKICK\_SAY\]\] \[-c \[CLEAR\_ME\]\] \[-m
\[MOVEME\]\] \[-i \[OBJECT\]\] \[-k\] \[-q\] \[-f \{fade,f,unfade,u\}\] \[-e
\[SET\_CORRECT\]\] \[-w \{show,s,hide,h\}\] \[-u \[SET\_ME\_UP\]\]

Optional arguments:

-h, --help
Show this help message and exit.

-l \[LOADME\], --load \[LOADME\]
Tell tablet to load the game object specified in the LOADME json config file.

-t \{enable,e,disable,d\}, --touch \{enable,e,disable,d\}
Enable or disable touch events on the tablet.

-r, --reset
Reload all objects and reset secene on the tablet.

-d \[SIDEKICK\_DO\], --sidekick\_do \[SIDEKICK\_DO\]
Tells virtual sidekick on tablet to do the specified action.

-s \[SIDEKICK\_SAY\], --sidekick\_say [SIDEKICK\_SAY]
Tells virtual sidekick on tablet to say the specified speech.

-c, \[CLEAR\_ME\], --clear \[CLEAR\_ME\]
Clear objects from game scene; if no string provided, clears all; otherwise
CLEAR\_ME is used to determine which objects to clear.

-m \[MOVEME\], --move \[MOVEME\]
Move specified game object to specified position.

-i \[OBJECT\], --highlight \[OBJECT\]
Highlight specified game object.

-k, --keyframe
Request state of all game objects on tablet.

-q, --quit
Quit the tablet app.

-f \{fade,f,unfade,u\}, --fade \{fade,f,unfade,u\}
Fade or unfade the tablet screen.

-e \[SET\_CORRECT\], --set\_correct \[SET\_CORRECT\]
Tag a set of objects as correct or incorrect responses.

-w \{show,s,hide,h\}, --correct \{show,s,hide,h\}
Show or hide visual feedback for correct or incorrect responses.

-u \[SET\_ME\_UP\], --setup\_scene \[SET\_ME\_UP\]
Set up initial game scene for social stories game.

-o \[STORY\_SELECTION\], --story-selection \[STORY\_SELECTION\]
Load a storybook.

-b \{show,s,hide,h\}, --buttons \{show,s,hide,h\}
Show/hide the flip buttons for a storybook.

-p \[STORY\_PAGE\], --story-go-to-page \[STORY\_PAGE\]
Go to a page in a storybook.

## JSON config files

Several example config files are included here. Each file should contain one
valid JSON object with the relevant properties for the object to load. For
example, if you want to load the "dragon" image as a draggable PlayObject, with
the audio file "chimes" attached (so that "chimes" plays whenever you tap the
object), and you want this object to be created at /(0,0,0/) in the tablet
screen world with the image scaled by /(100,100,100/), you would write:

> {    
> "name": "dragon",  
> "tag": "PlayObject",  
> "draggable": "true",  
> "audioFile": "chimes",  
> "position": [0,0,0],  
> "scale": [100,100,100]   
> }

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

If you want to set up a social stories game scene, you specify whether the scenes will be in order or not, the number of scenes in the story, and the number of answer options:
> {  
>   "numScenes":"4",  
>   "scenesInOrder":true,  
>   "numAnswers":"4"  
> }

## Version and dependency notes

This node was built and tested with:

- Python 2.7.6
- ROS Indigo
- Ubuntu 14.04 LTS (64-bit)
- sar\_opal\_msgs 4.0.0

## Bugs and issues

Please report all bugs and issues on the [sar\_opal\_sender github issues
page](https://github.com/personal-robots/sar_opal_sender/issues).
