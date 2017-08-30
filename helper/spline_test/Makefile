APP=spline_test

all: clear comp run

clear:
	rm -rf ${APP}.exe *.log

comp:
	g++ -Wall ${APP}.cpp -o ${APP}.exe

run:
	./${APP}.exe > pts.log
	python show.py
