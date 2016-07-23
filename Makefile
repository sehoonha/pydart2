build:
	python setup.py build build_ext

upload:
	python setup.py sdist upload

clean:
	rm -r _pydart2_api.so pydart2/pydart2_api.py build pydart2/*.pyc

