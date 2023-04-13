# Etch-A-Sketch SVG Renderer
This project allows users to take an SVG file, process it into a tool path, and send the tool path to an external microcontroller where it is drawn onto an Etch-A-Sketch. The SVG files should be kept in a `./svgs` folder. The project runs on Python 3.10.

## Overview
The project consists of the following files:

1. process.py: This file contains functions to process an SVG file into a tool path, which can then be sent to an external microcontroller.
2. master.py: This file contains functions to communicate with an external microcontroller and send the tool path created by process.py for rendering on the Etch-A-Sketch.
3. main.py: This is the entry point of the application. It provides a command-line interface for users to choose an SVG file from the ./svgs folder or download an SVG from a URL, and then process and render the SVG using the process.py and master.py modules.

## Installation
To set up the project, follow these steps:

1. Install Python 3.10 if you haven't already. You can download it from the official [Python website](https://www.python.org/downloads/).
2. Clone this repository or download and extract the project files to your local machine.
3. Open a terminal/command prompt and navigate to the project folder.
4. Run the following command to install the required libraries:
```bash
pip install -r requirements.txt
```

## Usage
To run the program, navigate to the project folder in a terminal/command prompt and run the following command:


```bash
python main.py
```

This will launch the command-line interface for the application. You will be presented with three options:

1. Look in the SVG path: This option allows you to select an SVG file from the ./svgs folder. The application will then process the SVG and send the tool path to the microcontroller for rendering on the Etch-A-Sketch.
2. Get from URL: This option allows you to input a URL of an SVG file. The application will download the SVG, process it, and send the tool path to the microcontroller for rendering on the Etch-A-Sketch.
3. Exit: This option exits the application.

Follow the on-screen prompts to use the application.