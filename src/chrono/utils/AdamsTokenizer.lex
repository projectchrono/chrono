%option nodefault
%option noyywrap
%option nounistd
%option never-interactive

%{
#include "chrono/utils/ChParserAdams.h"

bool in_comment = false;
%}

whitespace	[ \t]+
newline		\r?\n
comma		,
floatval	-?[[:digit:]]*\.[[:digit:]]*D?|-?[[:digit:]]+D?
label		[_[:alpha:]]+
equals		[=]
comment		!.*$
negate		"-"
colon		:
end			END
backslash	\\
adams		ADAMS\/
graphics	GRAPHICS\/
paren_o 	\(
paren_c 	\)
units		UNITS\/
marker		MARKER\/
part		PART\/
joint		JOINT\/
accgrav		ACCGRAV\/
request		REQUEST\/
output		OUTPUT\/

/* DELIMITER signals that an Adams object token stream is next */
%%
{whitespace}	/* don't do anything, we don't care */
{end}		ChParserAdams::tokens.emplace_back(DELIMITER, "|");ChParserAdams::tokens.emplace_back(END, yytext);
{newline}	/* Ignore newlines */ /*ChParserAdams::tokens.emplace_back(NEWLINE, yytext);*/
{comma}		/* comma's aren't important either */ /*ChParserAdams::tokens.emplace_back(COMMA, ",");*/
{floatval}	ChParserAdams::tokens.emplace_back(VALUE, yytext);
{label}		ChParserAdams::tokens.emplace_back(LABEL, yytext);
{equals}	/*ChParserAdams::tokens.emplace_back(EQUALS, "=");*/
{comment}	/* Ignore comments */ /*ChParserAdams::tokens.emplace_back(COMMENT, "!");*/
{negate}	ChParserAdams::tokens.emplace_back(NEGATE, "-");
{colon}		ChParserAdams::tokens.emplace_back(COLON, ":");
{backslash}	ChParserAdams::tokens.emplace_back(BACKSLASH, "\\");
{paren_o}	ChParserAdams::tokens.emplace_back(PAREN_O, "(");
{paren_c}	ChParserAdams::tokens.emplace_back(PAREN_C, ")");
{adams}		ChParserAdams::tokens.emplace_back(ADAMS, yytext);
{graphics}	ChParserAdams::tokens.emplace_back(DELIMITER, "|");ChParserAdams::tokens.emplace_back(GRAPHICS, yytext);
{units}		ChParserAdams::tokens.emplace_back(UNITS, yytext);
{marker}	ChParserAdams::tokens.emplace_back(DELIMITER, "|");ChParserAdams::tokens.emplace_back(MARKER, yytext);
{part}		ChParserAdams::tokens.emplace_back(DELIMITER, "|");ChParserAdams::tokens.emplace_back(PART, yytext);
{joint}		ChParserAdams::tokens.emplace_back(DELIMITER, "|");ChParserAdams::tokens.emplace_back(JOINT, yytext);
{accgrav}	ChParserAdams::tokens.emplace_back(DELIMITER, "|");ChParserAdams::tokens.emplace_back(ACCGRAV, yytext);
{request}	ChParserAdams::tokens.emplace_back(DELIMITER, "|");ChParserAdams::tokens.emplace_back(REQUEST, yytext);
{output}	ChParserAdams::tokens.emplace_back(OUTPUT, yytext);

%%
namespace chrono {
namespace utils {

void ChParserAdams::tokenize(const std::string& filename) {
	yyin = fopen(filename.c_str(), "r");
	if (yyin == NULL) {
		std::cout << "File " << filename << " could not be opened!" << std::endl;
		return;
	}

	ChParserAdams::yylex();
}

}
}
