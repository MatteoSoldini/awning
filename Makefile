main: main.c
	mkdir -p build

	cc \
		main.c \
		control/control.c \
		common/math/mat.c \
		common/math/geom.c \
		-o build/main \
		-Wall \
		-Wextra \
		-std=c99 \
		-g \
		-O0 \
		-Icontrol \
		-Icommon \
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
	cd build && ./main.exe