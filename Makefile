# ########################################33
# An incomplete set of useful commands...
SOURCES = $(wildcard pydart2/*.cpp)
HEADERS = $(wildcard pydart2/*.h)
INTERFACE = pydart2/pydart2_api.i

build: $(SOURCES) $(HEADERS) $(INTERFACE)
	python3 setup.py build build_ext
	python3 setup.py bdist_wheel
upload:
	python3 setup.py sdist upload

clean:
	rm -r *.so pydart2/pydart2_api.py build pydart2/pydart2_api_wrap.cpp pydart2/*.pyc

build2: $(SOURCES) $(HEADERS) $(INTERFACE)
	python2 setup.py build build_ext
	python2 setup.py bdist_wheel

upload2:
	python2 setup.py sdist upload

build3:  $(SOURCES) $(HEADERS) $(INTERFACE)
	python3 setup.py build build_ext
	python3 setup.py bdist_wheel

upload3:
	python3 setup.py sdist upload

runtests:
	python3 -m unittest discover -s tests -p "test*.py" -v

runtests2:
	python -m unittest discover -s tests -p "test*.py" -v

fix:
	sudo auditwheel repair dist/pydart2-0.7.4-cp35-cp35m-linux_x86_64.whl
	twine upload wheelhouse/pydart2-0.7.4-cp35-cp35m-manylinux1_x86_64.whl
