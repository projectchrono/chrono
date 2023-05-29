/* =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2017 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Colin Vanden Heuvel, Conlain Kelly
// =============================================================================
//
// Used to generate tokenizer for ChParserAdams. The lex file is used to
// generate ChParserAdamsTokenizer.yy.cpp via the command
//		`flex -o ChParserAdamsTokenizer.yy.cpp ParserAdamsTokenizer.lex`
// This file provides the funciton
// =============================================================================
*/
%option nodefault
%option noyywrap
%option nounistd
%option never-interactive

%top{
#include "chrono_parsers/ChParserAdams.h"
}

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
{end}		m_tokens.emplace_back(DELIMITER, "|");m_tokens.emplace_back(END, yytext);
{newline}	/* Ignore newlines */ /*m_tokens.emplace_back(NEWLINE, yytext);*/
{comma}		/* comma's aren't important either */ /*m_tokens.emplace_back(COMMA, ",");*/
{floatval}	m_tokens.emplace_back(VALUE, yytext);
{label}		m_tokens.emplace_back(LABEL, yytext);
{equals}	/*m_tokens.emplace_back(EQUALS, "=");*/
{comment}	/* Ignore comments */ /*m_tokens.emplace_back(COMMENT, "!");*/
{negate}	m_tokens.emplace_back(NEGATE, "-");
{colon}		m_tokens.emplace_back(COLON, ":");
{backslash}	m_tokens.emplace_back(BACKSLASH, "\\");
{paren_o}	m_tokens.emplace_back(PAREN_O, "(");
{paren_c}	m_tokens.emplace_back(PAREN_C, ")");
{adams}		m_tokens.emplace_back(ADAMS, yytext);
{graphics}	m_tokens.emplace_back(DELIMITER, "|");m_tokens.emplace_back(GRAPHICS, yytext);
{units}		m_tokens.emplace_back(UNITS, yytext);
{marker}	m_tokens.emplace_back(DELIMITER, "|");m_tokens.emplace_back(MARKER, yytext);
{part}		m_tokens.emplace_back(DELIMITER, "|");m_tokens.emplace_back(PART, yytext);
{joint}		m_tokens.emplace_back(DELIMITER, "|");m_tokens.emplace_back(JOINT, yytext);
{accgrav}	m_tokens.emplace_back(DELIMITER, "|");m_tokens.emplace_back(ACCGRAV, yytext);
{request}	m_tokens.emplace_back(DELIMITER, "|");m_tokens.emplace_back(REQUEST, yytext);
{output}	m_tokens.emplace_back(OUTPUT, yytext);

%%
namespace chrono {
namespace parsers {

// Opens the relevant file and calls yylex(), defined in ChParserAdamsTokenizer.yy.cpp
// Tokenizes the file into the member tokens vector.
// This file needs to be in the same file as yylex() is defined since it uses variables
// private to that file.
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
