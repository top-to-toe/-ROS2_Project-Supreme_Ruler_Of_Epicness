////////// No. 1 //////////
/* [[[ old C++ style ]]]
#include <iostream.h>

int main() {
	cout << "Hello, World!" << endl;
	return 0;
}
*/

////////// No. 2 //////////
/*
#include <iostream>

int main() {
	std::cout << "Hello, World!" << std::endl;
	return 0;
}
*/

////////// No. 3 //////////

#include <iostream>

using std::cout;
using std::endl;

int main() {
	cout << "Hello, World!" << endl;
	return 0;
}

////////// No. 4 //////////
/*		수업에서는 지양...
#include <iostream>

using namespace std;

int main() {
	cout << "Hello, World!" << endl;
	return 0;
}
*/
