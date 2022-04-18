#include "parser.hpp"
#include "Arduino.h"
#include <cstdint>

/* Removes luminaire ID from command string, leaving only its arguments.
 */
char *skipID(const char *command)
{
    char *after_luminaire_id = NULL;
    strtoll(command, &after_luminaire_id, 10); // remove luminaire id
    return after_luminaire_id;
}

bool _executeCommand(const char *command, Command *command_list, bool onlyCheckValid)
{
    while (isspace(*command))
        command++;
    for (Command *cur_command = command_list; cur_command->identifier != '\0'; cur_command++)
    {
        if (cur_command->identifier == command[0])
        {
            command++;
            if (cur_command->children != NULL)
            {
                if (!_executeCommand(command, cur_command->children, onlyCheckValid))
                    return false;
            }
            else
            {
                char *after_luminaire_id = skipID(command);
                if (after_luminaire_id == command)
                {
                    return false;
                }
                else
                {
                    if (onlyCheckValid)
                        return true;
                    cur_command->func(after_luminaire_id);
                }
            }
            return true;
        }
    }
    return false;
}

signed char _getLuminaireId(const char *command_str, Command *command_list)
{
    if (command_str[0] == '\0')
    {
        return -1;
    }
    while (isspace(command_str[0]))
    {
        command_str++;
    }
    for (unsigned int i = 0; command_list[i].identifier != '\0'; i++)
    {
        if (command_list[i].identifier == command_str[0])
        {
            if (command_list[i].children != NULL)
            {
                return _getLuminaireId(command_str + 1, command_list[i].children);
            }
        }
    }
    char *after_luminaire_id = NULL;
    signed char ID = strtol(command_str, &after_luminaire_id, 10);
    if (after_luminaire_id == command_str)
    {
        return -1;
    }
    return ID;
}

void _help(unsigned int depth, Command *command_list, char prefix[])
{
    char pre_prefix[4 * depth + 1];
    pre_prefix[4 * depth] = '\0';
    for (unsigned int i = 0; i < 4 * depth; i++)
    {
        pre_prefix[i] = ' ';
    }
    for (unsigned int i = 0; command_list[i].identifier != '\0'; i++)
    {
        prefix[2 * depth] = '\0';
        if (command_list[i].children != NULL)
        {
            Serial.printf("%s%s%c %s: %s\n", pre_prefix, prefix, command_list[i].identifier, "<see subcommands>", command_list[i].help_text);
        }
        else
        {
            Serial.printf("%s%s%c <i> %s: %s\n", pre_prefix, prefix, command_list[i].identifier, command_list[i].usage_format, command_list[i].help_text);
        }
        if (command_list[i].children != NULL)
        {
            char prefix_elongation[] = {command_list[i].identifier, ' ', '\0'};
            char prefix_elongated[4 * depth + 1] = {'\0'};
            strcpy(prefix_elongated, prefix);
            _help(depth + 1, command_list[i].children, strcat(prefix_elongated, prefix_elongation));
        }
    }
}

void noOpt(const char *args)
{
    for (unsigned int i = 0; args[i] != '\0'; i++)
        if (!isspace(args[i]))
        {
            Serial.printf("Command has no options. \"%s\" ignored\n", args);
            return;
        }
}

/* Returns true if there is a matching command (and executes it)
 */
bool CommandParser::executeCommand(const char *command)
{
    bool onlyCheckValid = false;
    return _executeCommand(command, command_list, onlyCheckValid);
}

/* Returns true if there is a matching command
 */
bool CommandParser::validCommand(const char *command)
{
    bool onlyCheckValid = true;
    return _executeCommand(command, command_list, onlyCheckValid);
}

/* Retrieves luminaire ID from command string, and returns it.
 */
signed char CommandParser::getLuminaireId(const char *command)
{
    return _getLuminaireId(command, command_list);
}

void CommandParser::help()
{
    Serial.printf("The first argument after the command identifier (<i>) is the target luminaire id.\n");
    Serial.printf("Indented commands are subcommands of the last unindented command.\n");
    Serial.printf("Spaces are only required between numbers. They are otherwise ignored.\n");
    Serial.printf("Available commands:\n");
    char prefix[] = {'\0'};
    _help(0, command_list, prefix);
}

const char *CommandParser::strip(const char *command)
{
    unsigned int idx_command=0, idx_stripped=0;    

    // skip all blank characters until luminaire id
    // command is not assumed to be valid, hence '\0' check
    while((command[idx_command] != '\0') && (!isdigit(command[idx_command])))
    {
        if (!isblank(command[idx_command]))
        {
            stripped[idx_stripped] = command[idx_command];
            idx_stripped++;
        }
        idx_command++;
    }

    return (const char *)stripped;
}
