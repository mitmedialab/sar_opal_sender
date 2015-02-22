# sar\_opal\_sender
Opal sender is a ROS package that allows you to enter commands to send to a SAR Opal tablet via a rosbridge\_server websocket connection.

## Usage
opal\_sender.ph \[-h\] \[-l \[LOADME\]\] \[-t \{enable,e,disable,d\}\]\[-r\]\[-a \[SIDEKICK\_DO\]\] \[-s \[SIDEKICK\_SAY\]\]

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

-s [SIDEKICK\_SAY], --sidekick\_say [SIDEKICK\_SAY]
tells virtual sidekick on tablet to say the specified speech

## Load object JSON config files
Several example config files are included here. Each file should contain one valid JSON object with the relevant properties for the object to load. For example, if you want to load the "dragon" image as a draggable PlayObject, with the audio file "chimes" attached (so that "chimes" plays whenever you tap the object), and you want this object to be created at /(0,0,0/) in the tablet screen world, you would write:
> /{
>    "name": "dragon",
>    "tag": "PlayObject",
>    "draggable": "true",
>    "audioFile": "chimes",
>    "initPosition": ["0","0","0"]
> /}

If you wanted to specify that the object has a target end point (i.e., something special should happen if the object reaches a target position on the screen), you would add the field "endPositions":
> /{
>    "name": "dragon",
>    "tag": "PlayObject",
>    "draggable": "true",
>    "audioFile": "chimes",
>    "initPosition": ["0","0","0"],
>    "endPositions": [ ["100", "200", "0"] ]
> /}

Note that here, "endPositions" is an array of arrays, because you can specify *multiple* target positions.

If you want to load a background image instead of a draggable object, you would specify just the name of the image and the tag:
> /{
>    "name": "playground",
>    "tag": "Background"
> /}

