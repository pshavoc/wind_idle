.PHONY: all clean build

all: build

build:
	cd windpark && wasm-pack build --target web