PYTHON ?= python
MODULE = search
INPUT = test-vis1.csv

run:
	$(PYTHON) -m $(MODULE) < $(INPUT)
