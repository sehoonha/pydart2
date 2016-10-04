# ########################################33
# An incomplete set of useful commands...
SOURCES = $(wildcard pydart2/*.cpp)
HEADERS = $(wildcard pydart2/*.h)
INTERFACE = pydart2/pydart2_api.i

build: $(SOURCES) $(HEADERS) $(INTERFACE)
	python3 setup.py build build_ext

upload:
	python3 setup.py sdist upload

clean:
	rm -r *.so pydart2/pydart2_api.py build pydart2/pydart2_api_wrap.cpp pydart2/*.pyc

build2: $(SOURCES) $(HEADERS) $(INTERFACE)
	python2 setup.py build build_ext

upload2:
	python2 setup.py sdist upload

build3:  $(SOURCES) $(HEADERS) $(INTERFACE)
	python3 setup.py build build_ext

upload3:
	python3 setup.py sdist upload

runtests:
	python3 -m unittest discover -s tests -p "test*.py" -v

runtests2:
	python -m unittest discover -s tests -p "test*.py" -v
