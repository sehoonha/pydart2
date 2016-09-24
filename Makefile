# ########################################33
# An incomplete set of useful commands...

build: pydart2/pydart2_api.h pydart2/pydart2_api.cpp pydart2/pydart2_draw.h pydart2/pydart2_draw.cpp pydart2/pydart2_api.i
	python3 setup.py build build_ext

upload:
	python3 setup.py sdist upload

clean:
	rm -r *.so pydart2/pydart2_api.py build pydart2/pydart2_api_wrap.cpp pydart2/*.pyc

build2: pydart2/pydart2_api.h pydart2/pydart2_api.cpp pydart2/pydart2_draw.h pydart2/pydart2_draw.cpp pydart2/pydart2_api.i
	python2 setup.py build build_ext

upload2:
	python2 setup.py sdist upload

build3: pydart2/pydart2_api.h pydart2/pydart2_api.cpp pydart2/pydart2_draw.h pydart2/pydart2_draw.cpp pydart2/pydart2_api.i
	python3 setup.py build build_ext

upload3:
	python3 setup.py sdist upload
