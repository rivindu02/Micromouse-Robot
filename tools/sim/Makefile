# Host build of the maze simulator. Uses the real firmware sources in CORE.
#   make            build build/sim
#   make run        run on 200 generated mazes
#   make gif        regenerate docs/exploration.gif (needs Python + Pillow)
#   make CORE=/path/to/other/checkout/Core   compare another version

CORE   ?= ../../Core
CC     ?= gcc
CFLAGS ?= -O2 -g -Wall -Wno-unused-function -fsanitize=address,undefined
SRCS    = sim_main.c $(CORE)/Src/micromouse.c $(CORE)/Src/communication.c \
          $(CORE)/Src/audio.c $(CORE)/Src/utils.c

build/sim: $(SRCS) stub/stm32f4xx_hal.h
	@mkdir -p build
	$(CC) $(CFLAGS) $(EXTRA) -Istub -I$(CORE)/Inc $(SRCS) -lm -o $@

run: build/sim
	./build/sim -v --random 200 1

gif: build/sim
	./build/sim --random 1 27 --trace build/trace.txt
	python3 render_gif.py build/trace.txt ../../docs/exploration.gif

clean:
	rm -rf build

.PHONY: run gif clean
