#include <iostream>
#include "sml_Client.h"

using namespace std;
using namespace sml;

int main(int argc, char *argv[]) {
    Kernel *k = Kernel::CreateKernelInNewThread();
    Agent *a = k->CreateAgent("soar");
    cout << a->ExecuteCommandLine("echo Hello World") << endl;

    string dummy;
    cin >> dummy;
    return 0;
}
