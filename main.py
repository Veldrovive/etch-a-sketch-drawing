from process import get_control_path_from_svg
from master import find_arduino_port, send_connect, send_initial_position, draw, send_end_drawing

import serial
import click
import os
import time
import requests
from pathlib import Path

SVG_PATH = Path("./svgs")

def process_svg_file(svg_file_path: Path):
    # Add your function implementation here
    print(f"Processing {svg_file_path}")
    arduino_port = find_arduino_port()
    ser = serial.Serial(arduino_port, 9600)

    start_position, control_path = get_control_path_from_svg(svg_file_path)

    # Connect and wait for the Arduino to be ready for a new drawing
    send_connect(ser)

    # Set initial position (x, y)
    send_initial_position(ser, start_position.real, start_position.imag)

    # Clear the screen and wait for use input to actually start drawing
    click.clear()
    input("If it is not clear, shake the tablet to clear it. Press enter to start drawing:")

    # Send the control path
    draw(ser, control_path)

    # End drawing so we can start a new one
    send_end_drawing(ser)

    ser.close()

@click.command()
def main():
    while True:
        click.clear()
        choice = click.prompt("Choose an option\n1. Look in the SVG path\n2. Get from URL\n3. Exit", type=int)
        
        if choice == 1:
            click.clear()
            svg_files = [f for f in os.listdir(SVG_PATH) if f.endswith(".svg")]
            for idx, svg_file in enumerate(svg_files, start=1):
                click.echo(f"{idx}. {svg_file[:-4]}")
            
            file_index = click.prompt("Choose a file by number", type=int)
            chosen_file = Path(SVG_PATH) / svg_files[file_index - 1]
            process_svg_file(chosen_file)

        elif choice == 2:
            click.clear()
            url = click.prompt("Enter the SVG URL")
            response = requests.get(url)
            tmp_svg = Path("tmp.svg")
            with open(tmp_svg, "wb") as f:
                f.write(response.content)
            process_svg_file(tmp_svg)
            tmp_svg.unlink()

        elif choice == 3:
            click.echo("Goodbye!")
            break

        else:
            click.echo("Invalid option. Please try again.")

if __name__ == "__main__":
    main()