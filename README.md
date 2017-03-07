# Ev3 print3rbot

Python code for the EV3-Print3rbot, based on the [ev3dev project](http://ev3dev.org).

## Installation

### Dependencies

The ev3dev version must be at least [ev3dev-jessie-ev3-generic-2017-02-11](https://github.com/ev3dev/ev3dev/releases/download/ev3dev-jessie-2017-02-11/ev3dev-jessie-ev3-generic-2017-02-11.zip).

The code uses the [ev3dev Python3 API](https://github.com/rhempel/ev3dev-lang-python) from @rhempel and @ddemidov. It is already installed on the ev3dev distribution.

### EV3-Print3rbot code

To put the EV3-Print3rbot code  on your EV3, clone this repository:
```
git clone https://github.com/cavenel/ev3-print3rbot.git
cd ev3-print3rbot/
```
You can then launch the robot using
```
python3 writer.py
```
or make sure the file is executable and launch it directly:
```
chmod +x writer.py
./writer.py
```
EV3-Print3rbot can then be launched from brickman via the file explorer.

## Usage

### Configuration

The file `writer.py` contains four parameters that may need to be tuned if you did not follow precisely the building instructions: `xA`, `yA`, `xB`, `yB`, `r1` and `r2`. They define the position and length of the robots arms:
```
          .E      The pen is at coordinates E = (xE,yE)
         / \      r1 = CE = DE = 17.3125 lego units.
        /   \     r2 = AC = BD = 11 lego units.
       /     \    A  = (0,0)
     C.       .D  B  = (6,0)
       \     /
        \   /
        A. .B
       -------
       [robot]
       -------
```

### Drawing an SVG image

The EV3 Print3rbot can draw directly from SVG images, using the [svg.path](https://pypi.python.org/pypi/svg.path) library (included in sources). The images must contain only paths without transformations. To convert any SVG image, you can use inkscape in command line:
```
inkscape --verb=EditSelectAll --verb=ObjectToPath --verb=SelectionUnGroup --verb=FileSave --verb=FileClose --verb=FileQuit myfile.svg
```
Make sure that Inkscape is configured with "Transforms -> Store transformation" set to "Optimized" to remove all transformations in the process.

The image will be automatically resized to fit in the robot drawing area. To draw an image, use the code:
```
wri = Writer(calibrate = True)
wri.draw_image(image_file = 'images/test.svg')
```
The image must be uploaded on the EV3 first. You can use any SFTP client program to transfer files, like WinSCP (Windows) or NetworkManager (Ubuntu). 

### Using mouse as input

The robot can use mouse as input. The code is based on the [python-evdev package](https://python-evdev.readthedocs.io/en/latest/) (already installed on ev3dev). To use mouse as input, use the code:

```
wri = Writer(calibrate = True)
wri.follow_mouse()
```

## Building instructions

See [the project page](http://www.ev3dev.org/projects/2015/05/06/EV3-Print3rbot/) for building instructions.
