PY ?= python3
.PHONY: figures check clean
figures:
	$(PY) analysis/check_report.py
	$(PY) analysis/fig_response.py
	$(PY) analysis/fig_poles_qr.py
	$(PY) analysis/fig_filter.py
	$(PY) analysis/fig_hero.py
	$(PY) analysis/fig_filter_anim.py
	$(PY) analysis/fig_diagrams.py
clean:
	rm -f media/*.png media/*.gif media/*.svg
