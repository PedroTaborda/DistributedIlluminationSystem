#ifndef PARSER_HPP
#define PARSER_HPP

void noOpt(const char *args);

#define UNUSED(x) (void)(x) // removes unused parameter warning - useful when intentional
#define noArgCom(funcNoArgs) [](const char *args) { noOpt(args); funcNoArgs(); }

#define floatArgCom(funcFloatArg) [](const char *args) { funcFloatArg(atof(args)); }
#define intArgCom(funcIntArg) [](const char *args) { funcIntArg(atoi(args)); }
#define boolArgCom(funcBoolArg) [](const char *args) { funcBoolArg(atoi(args)); }

#define printfCom(...) noArgCom([]() { Serial.printf(__VA_ARGS__); })

#define notImplemented [](const char *args) { UNUSED(args); Serial.println("Not implemented\n"); }

#define ACK Serial.printf("ack\n");
#define ERR Serial.printf("err\n");

struct Command
{
    char identifier;
    const char *usage_format;
    const char *help_text;
    void (*func)(const char *);
    Command *children;
};

class CommandParser
{
public:
    CommandParser(){command_list = nullptr;};
    CommandParser(Command *command_list) : command_list(command_list){};

    bool executeCommand(const char *command);

    bool validCommand(const char *command);

    const char *strip(const char *command);

    signed char getLuminaireId(const char *command);

    void help();

private:
    Command *command_list;
    char stripped[128];
};


#endif // PARSER_HPP