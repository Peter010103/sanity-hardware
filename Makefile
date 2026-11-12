CC ?= cc
CFLAGS = -O2 -Wall -Wextra -Werror -shared -fPIC

.PHONY: native test clean
native: tools/libhx711.so tools/libhx711_slow10.so

tools/libhx711.so: tools/hx711_native.c
	$(CC) $(CFLAGS) -o $@ $<

tools/libhx711_slow10.so: tools/hx711_slow10.c
	$(CC) $(CFLAGS) -o $@ $<

test:
	python3 -m pytest -q tools

clean:
	rm -f tools/libhx711.so tools/libhx711_slow10.so
