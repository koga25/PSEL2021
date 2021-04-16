### PSEL2021

This repository represents a foothold to begin the development of the 2021 Maracatronics' Selection Program.

## Requirements:

    - g++ (used v9.3.0)
    - Qt (used version v5.12.8)
    - Qt OpenGL
    - Google protocol buffers (used protoc v3.6.1)


## Compile instruction:
Create a folder named build, open it and run the command `qmake ..`
Afterwards run the command `make` while still inside the build folder.
If built correctly, there will be a new folder named `bin` in the project.

```
mkdir build
cd build
qmake ..
make
cd ..
ls
```

