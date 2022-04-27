#ifndef PARSER_HPP
#define PARSER_HPP

#include "stdint.h"

#define UNUSED(x) (void)(x) // removes unused parameter warning - useful when intentional
#define noArgCom(funcNoArgs) [](const char *args, uint8_t dsp) -> char * { UNUSED(args); UNUSED(dsp); return funcNoArgs(); }

#define floatArgCom(funcFloatArg) [](const char *args, uint8_t dsp) -> char * { UNUSED(dsp); return funcFloatArg(atof(args)); }
#define intArgCom(funcIntArg) [](const char *args, uint8_t dsp) -> char * { UNUSED(dsp); return funcIntArg(atoi(args)); }
#define boolArgCom(funcBoolArg) [](const char *args, uint8_t dsp) -> char * { UNUSED(dsp); return funcBoolArg(atoi(args)); }
#define idArgCom(funcIdArg) [](const char *args, uint8_t dsp) -> char * { UNUSED(args); return funcIdArg(dsp); }

#define printI2C(...) {static char print_buf[128]; snprintf(print_buf, 128, __VA_ARGS__); return print_buf; }

#define printfCom(...) noArgCom([]() {printI2C(__VA_ARGS__)})

extern char NOT_IMPLEMENTED_BUF[];
extern char ACK_BUF[];
extern char ERR_BUF[];

#ifdef DEBUG
#define notImplemented noArgCom([]() {_Pragma("message(\"Command not implemented\")"); return NOT_IMPLEMENTED_BUF; })
#else
#define notImplemented noArgCom([]() {return NOT_IMPLEMENTED_BUF; })
#endif

#define ACK return ACK_BUF;
#define ERR return ERR_BUF;

struct Command
{
    char identifier;
    const char *usage_format;
    const char *help_text;
    char *(*func)(const char *, uint8_t);
    Command *children;
};

class CommandParser
{
public:
    CommandParser(){command_list = nullptr;};
    CommandParser(Command *command_list);

    char *executeCommand(const char *command, uint8_t displayerID) volatile;

    const char *strip(const char *command) volatile;

    signed char getLuminaireId(const char *command) volatile;

    Command *command_list;
private:
    char stripped[128];
};

char *help();

#endif // PARSER_HPP