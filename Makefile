build: pydart2/pydart2_api.h pydart2/pydart2_api.cpp pydart2/pydart2_draw.h pydart2/pydart2_draw.cpp pydart2/pydart2_api.i
	python setup.py build build_ext

upload:
	python setup.py sdist upload

clean:
	rm -r _pydart2_api.so pydart2/pydart2_api.py build pydart2/pydart2_api_wrap.cpp pydart2/*.pyc

