"""Build the RT1064 camera target with the installed Arm Compiler 6 tools."""

import argparse
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

from extract_replay import ROOT


def build(project, output, tools, shadow):
    output.mkdir(parents=True, exist_ok=True)
    target = ET.parse(project).find("./Targets/Target")
    arm = target.find("./TargetOption/TargetArmAds")
    options = arm.find("./Cads/VariousControls")
    linker = arm.find("LDads")
    files = target.findall("./Groups/Group/Files/File")
    sources = [node.findtext("FilePath") for node in files if node.findtext("FileType") in ("1", "2")]
    libraries = [node.findtext("FilePath") for node in files if node.findtext("FileType") == "4"]
    objects = [output / (Path(source).stem.lower() + ".o") for source in sources]
    if len(set(objects)) != len(objects):
        raise ValueError("Target has conflicting object basenames")
    defines = ["-D" + re.sub(r"\s*=\s*", "=", v.strip()) for v in options.findtext("Define").split(",")]
    if shadow:
        defines.append("-DOBSTACLE_SHADOW_ENABLED=1")
    includes = ["-I" + v for v in options.findtext("IncludePath").split(";") if v]
    cpu = ["--target=arm-arm-none-eabi", "-mcpu=cortex-m7", "-mfpu=fpv5-d16", "-mfloat-abi=hard"]
    # Match the bundled vendor library ABI. The SDK uses an include whose
    # case differs on Windows, so use the case-insensitive target convention.
    c_flags = ["-O2", "-g", "-std=c99", "-ffunction-sections", "-fshort-wchar",
               "-fshort-enums", "-Wno-nonportable-include-path"]
    c_flags += options.findtext("MiscControls").split() + defines + includes
    asm_flags = ["-x", "assembler-with-cpp", "-DDEBUG",
                 "-D__STARTUP_INITIALIZE_NONCACHEDATA", "-D__MICROLIB"]

    def invoke(tool, flags, cwd=project.parent):
        process = subprocess.run([str(tools / (tool + ".exe")), *map(str, flags)],
                                 cwd=cwd, capture_output=True, text=True, errors="replace")
        messages = process.stdout + process.stderr
        if process.returncode:
            raise RuntimeError(f"{tool} exited {process.returncode}\n{messages}")
        return messages

    def compile_file(source, obj):
        flags = asm_flags if Path(source).suffix.lower() == ".s" else c_flags
        return invoke("armclang", cpu + flags + ["-c", source, "-o", obj])

    with ThreadPoolExecutor(max_workers=8) as pool:
        messages = "".join(pool.map(compile_file, sources, objects))
    print(f"Compiled {len(sources)} sources (shadow={int(shadow)})")
    if messages:
        print(messages, end="")
    # armlink 6.21 cannot open some inputs through a non-ASCII project path.
    linker_inputs = [project.parent / name for name in [linker.findtext("ScatterFile"), *libraries]]
    scatter, *local_libraries = [output / source.name for source in linker_inputs]
    for source, local in zip(linker_inputs, [scatter, *local_libraries]):
        shutil.copyfile(source, local)
    elf = output / "rt1064.axf"
    print(invoke("armlink", [
        "--cpu=Cortex-M7.fp.dp", "--library_type=microlib", "--strict",
        "--diag_suppress", linker.findtext("DisabledWarnings"),
        "--scatter", scatter, *linker.findtext("Misc").split(),
        "--summary_stderr", "--info=summarysizes,sizes,totals", "--map", "--symbols",
        "--list", output / "rt1064.map", "-o", elf, *objects, *local_libraries,
    ], cwd=output), end="")
    invoke("fromelf", ["--i32combined", "--output", output / "rt1064.hex", elf], cwd=output)
    print(output / "rt1064.hex")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--project", type=Path,
                        default=ROOT.parents[1] / "firmware/camera_car/demo/mdk/rt1064.uvprojx")
    parser.add_argument("--arm-bin", type=Path, default=Path("D:/Keil5/ARM/ARMCLANG/bin"))
    parser.add_argument("--shadow", action="store_true")
    args = parser.parse_args()
    build(args.project.resolve(), args.output.resolve(), args.arm_bin.resolve(), args.shadow)
