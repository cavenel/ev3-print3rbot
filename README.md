# Ev3 print3rbot

Python code for the EV3-Print3rbot, based on the [ev3dev project](http://ev3dev.org).

## Dependencies

The ev3dev version must be at least [ev3dev-jessie-2015-05-01](https://github.com/ev3dev/ev3dev/releases/tag/ev3dev-jessie-2015-05-01).

The code uses the [ev3dev Python API](https://github.com/ddemidov/ev3dev-lang-python) from @ddemidov. To install it:

* Prerequisites:
```
apt-get install libboost-python-dev python-setuptools python-pil
```

* Now, the actual module installation:
```
easy_install http://ddemidov.github.io/ev3dev-lang-python/python_ev3dev-latest.egg
```

## Usage

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

### Using mouse as input

The robot can use mouse as input. The code is based on the [evdev package](https://pypi.python.org/pypi/evdev) (included in the source). To use mouse as input, use the code:

```
wri = Writer(calibrate = True)
wri.follow_mouse()
```

## Building instructions

See [the project page](http://www.ev3dev.org/projects/2015/05/06/EV3-Print3rbot/) for building instructions.
