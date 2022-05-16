#define speakf printf
class emic2 : public Stream
{
public :
    emic2(PinName tx, PinName rx): _cmd(tx,rx) {
        _cmd.baud(9600);
        _cmd.putc('X'); //stop talking if reset and not a power on
        _cmd.putc('\r'); // Send a CR in case the system is already up
        wait(1); //delay for emic power on boot or reset respone
        while (_cmd.getc() != ':');   // When the Emic 2 has initialized and is ready, it will send a single ':'
        while (_cmd.readable()) _cmd.getc();//flush out buffer just in case
    };
    void ready() {
        while (_cmd.getc() != ':');
        while (_cmd.readable()) _cmd.getc();//flush out recieve buffer just in case
    };
    int readable() {
        return _cmd.readable();
    };
    int getc() {
        return _cmd.getc();
    }
    void volume(int x) {
        speakf("V%D\r",x);
        ready();
    }
    void voice(int x) {
        speakf("N%D\r",x);
        ready();
    }
protected :
    Serial     _cmd;
    //used by printf - supply it and printf works!
    virtual int _putc(int c) {
        _cmd.putc(c);
        return 0;
    };
    virtual int _getc() {
        return -1;
    };
};