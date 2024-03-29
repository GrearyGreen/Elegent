helloworld : main.o A.o B.o
	g++ -o helloworld main.o A.o B.o
main.o : main.cpp A.h B.h
	g++ -g -c main.cpp
A.o : A.cpp A.h
	g++ -g -c A.cpp
B.o : B.cpp B.h
	g++ -g -c B.cpp
clean:
<<<<<<< HEAD
	rm main.o A.o B.o helloworld
=======
	rm helloworld main.o A.o B.o
>>>>>>> a07557c8268ff26a6ed4c2f22c09c45e8580efa1
