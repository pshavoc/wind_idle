.PHONY: all clean build run

all: build

build:
	cd windpark && wasm-pack build --target web

run: build
	python3 -m http.server