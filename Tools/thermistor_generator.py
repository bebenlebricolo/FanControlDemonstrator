#!/usr/bin/python

from io import TextIOWrapper
import sys
import math
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from argparse import ArgumentParser

# Generates thermistor data c source files that exposes thermistor data
C_THERMISTOR_MAX_SAMPLES = 50
C_MIN_TEMP = -24
C_MAX_TEMP = 100
C_TEMP_STEP = 1
C_KELVIN_TO_DEG = 273.15
C_CALIB_TEMP = 25 + C_KELVIN_TO_DEG

@dataclass
class TempRes:
    temperature : float = 0
    resistance : float = 0

class ResistanceUnit(Enum):
    Ohms = "RESUNIT_OHMS"
    KiloOhms = "RESUNIT_KILOOHMS"
    MegaOhms = "RESUNIT_MEGAOHMS"

@dataclass
class ThermistorData :
    data : list[TempRes] = field(default_factory=list)
    unit : ResistanceUnit = ResistanceUnit.KiloOhms
    sample_count : int = 0

def write_cplusplus_extern_begin(file : TextIOWrapper) :
    file.write("#ifdef __cplusplus\n")
    file.write("extern \"C\" {\n")
    file.write("#endif\n\n")

def write_cplusplus_extern_end(file : TextIOWrapper) :
    file.write("#ifdef __cplusplus\n")
    file.write("}\n")
    file.write("#endif\n\n")

def generate_header(filepath : Path, name : str, sample_count : int) -> None :
    header_define = name.upper() + "_HEADER"
    print(f"   -> Writing header file {filepath} ... ")

    with open(filepath, "w") as file :
        file.write(f"#ifndef {header_define}\n")
        file.write(f"#define {header_define}\n\n")
        write_cplusplus_extern_begin(file)
        file.write(f"#include \"thermistor.h\"\n\n")
        file.write(f"#define {name.upper()}_SAMPLE_COUNT {sample_count}U\n\n")
        file.write(f"extern const thermistor_data_t {name}_data;\n\n")

        write_cplusplus_extern_end(file)
        file.write(f"#endif /* {header_define} */\n")

def generate_data(r0 : int, beta : int, count : int, min_temp : int, max_temp : int) -> ThermistorData :
    thermistor_data = ThermistorData()
    delta = (max_temp - min_temp) / count
    interval : list[int] = []

    for i in range(count + 1) :
        # Removes int aliasing -> round to nearest integer
        tmp = round(min_temp + i*delta)
        interval.append(int(tmp))

    for i in interval :
        data = TempRes()
        data.temperature = i + C_KELVIN_TO_DEG
        data.resistance = r0 * math.exp(beta * (1/(i + C_KELVIN_TO_DEG) - 1/C_CALIB_TEMP))
        thermistor_data.data.append(data)

    thermistor_data.sample_count = len(thermistor_data.data)

    # Raise a warning at the end, user might want to adjust parameters
    # or tweak C file him/herself
    if thermistor_data.sample_count >= C_THERMISTOR_MAX_SAMPLES :
        print(f"/!\\ too many samples in thermistor_data.data")
        print(f"    - samples_count : {len(interval)}")
        print(f"    - min temp : {min_temp}")
        print(f"    - max temp : {max_temp}")

    return thermistor_data

def generate_source_file(filepath : Path, name : str, data : ThermistorData) -> None :
    print(f"   -> Writing source file {filepath} ... ")
    with open(filepath, "w") as file :
        file.write(f"#include \"{name}.h\"\n\n")
        file.write(f"const thermistor_data_t {name}_data =  {{\n")
        file.write("    .data = {\n")
        for i in range(len(data.data)):
            file.write(f"        {{{int(data.data[i].temperature - C_KELVIN_TO_DEG)}, {int(data.data[i].resistance)}}}")
            if i < len(data.data) - 1:
                file.write(",")
            file.write("\n")
        file.write("    },\n")
        file.write(f"    .unit = {ResistanceUnit.KiloOhms.value},\n")
        file.write(f"    .sample_count = {name.upper()}_SAMPLE_COUNT\n")
        file.write("};\n")

def main(args : list[str]) -> int:
    script_dir = Path(__file__).parent.parent
    output_directory = script_dir.joinpath("Generated")
    if not output_directory.exists() :
        output_directory.mkdir(parents=True, exist_ok=True)

    parser = ArgumentParser()
    parser.add_argument("r0", help="Base resistance value of NTC thermistor (in KOhms)")
    parser.add_argument("beta", help="Thermistor thermal constant (β constant)")
    parser.add_argument("name", help="Name of the output file (e.g. \"thermistor_100k_3950K\")")
    parser.add_argument("--min", help="Minimum temperature (°Celsius) - integer")
    parser.add_argument("--max", help="Maximum temperature (°Celsius) - integer")
    parser.add_argument("--count", help="Number of desired data values in generated source file")

    parsed = parser.parse_args(args)
    r0 = int(parsed.r0)
    beta = int(parsed.beta)
    name = parsed.name

    try:
        min_temp = int(parsed.min)
        max_temp = int(parsed.max)
        count = int(parsed.count)
    except :
        print("/!\\ Caught invalid data in either min, max or count parameters. Expecting integer data type.")
        return 1

    if (max_temp - min_temp) < count :
        print("/!\\ Too many values will be generated !")
        print(f"     -> count {count} was given (or infered) but (max - min) = {max_temp - min_temp}")
        print(f"     -> there will potentially be several identical data points in the resulting file (due to integer aliasing)")
        print(f"     -> Clamping count to {max_temp - min_temp}")


    print("1 - Generating thermistor data")
    thermistor_data = generate_data(r0, beta, count - 1, min_temp, max_temp)
    print("2 - Generating header file")
    generate_header(output_directory.joinpath(name + ".h"), name, len(thermistor_data.data))
    print(f"3 - Generating source file in {output_directory}")
    generate_source_file(output_directory.joinpath(name + ".c"), name, thermistor_data)

    return 0





if __name__ == "__main__" :
    exit(main(sys.argv[1:]))