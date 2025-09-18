main: main.c
	if not exist build mkdir build

	cc \
		main.c \
		-o build/main \
		-Wall \
		-Wextra \
		-std=c99 \
		-g \
		-O0 \
		-IC:/raylib/raylib/src \
		-IC:/raylib/raylib/src/external \
		-LC:/raylib/raylib/src \
		-lraylib \
		-lopengl32 \
		-lgdi32 \
		-lwinmm \
		-Iinclude \
		-DRAYGUI_IMPLEMENTATION

run: main
	cd build && main.exe