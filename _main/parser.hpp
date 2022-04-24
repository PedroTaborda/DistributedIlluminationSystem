#ifndef PARSER_HPP
#define PARSER_HPP

#define UNUSED(x) (void)(x) // removes unused parameter warning - useful when intentional
#define noArgCom(funcNoArgs) [](const char *args) -> char * { UNUSED(args); return funcNoArgs(); }

#define floatArgCom(funcFloatArg) [](const char *args) -> char * { return funcFloatArg(atof(args)); }
#define intArgCom(funcIntArg) [](const char *args) -> char * { return funcIntArg(atoi(args)); }
#define boolArgCom(funcBoolArg) [](const char *args) -> char * { return funcBoolArg(atoi(args)); }

#define printI2C(...) {static char print_buf[128]; snprintf(print_buf, 128, __VA_ARGS__); return print_buf; }

#define printfCom(...) noArgCom([]() {printI2C(__VA_ARGS__)})

extern char NOT_IMPLEMENTED_BUF[];
extern char ACK_BUF[];
extern char ERR_BUF[];
#define notImplemented [](const char *args) { UNUSED(args); return NOT_IMPLEMENTED_BUF; }
#define ACK return ACK_BUF;
#define ERR return ERR_BUF;

struct Command
{
    char identifier;
    const char *usage_format;
    const char *help_text;
    char *(*func)(const char *);
    Command *children;
};

class CommandParser
{
public:
    CommandParser(){command_list = nullptr;};
    CommandParser(Command *command_list);

    char *executeCommand(const char *command) volatile;

    const char *strip(const char *command) volatile;

    signed char getLuminaireId(const char *command) volatile;

    Command *command_list;
private:
    char stripped[128];
};

char *help();

#endif // PARSER_HPP