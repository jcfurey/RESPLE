// Thin entry point for the RESPLE executable. The node implementation lives in
// RESPLE.cpp, compiled once into libresple; this wrapper just forwards to its
// exported entry function so the heavy source isn't compiled a second time.
int respleMain(int argc, char *argv[]);

int main(int argc, char *argv[])
{
    return respleMain(argc, argv);
}
