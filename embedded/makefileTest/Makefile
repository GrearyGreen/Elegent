helloworld : main.o A.o B.o
	g++ -o helloworld main.o A.o B.o
main.o : main.cpp A.h B.h
	g++ -g -c main.cpp
A.o : A.cpp A.h
	g++ -g -c A.cpp
B.o : B.cpp B.h
	g++ -g -c B.cpp

