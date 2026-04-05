using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;

namespace RoboForge_WPF.DSL
{
    public class Lexer
    {
        private readonly string _input;
        private int _position;
        private int _line = 1;
        private int _column = 1;

        private static readonly Dictionary<string, TokenType> Keywords = new(StringComparer.OrdinalIgnoreCase)
        {
            {"movej", TokenType.MOVEJ},
            {"movel", TokenType.MOVEL},
            {"while", TokenType.WHILE},
            {"if", TokenType.IF},
            {"else", TokenType.ELSE},
            {"wait", TokenType.WAIT},
            {"set_io", TokenType.SETIO},
            {"read_io", TokenType.READIO},
            {"stop", TokenType.STOP},
            {"return", TokenType.RETURN},
            {"true", TokenType.BOOLEAN},
            {"false", TokenType.BOOLEAN}
        };

        public Lexer(string input)
        {
            _input = input ?? string.Empty;
        }

        public List<Token> Tokenize()
        {
            var tokens = new List<Token>();
            Token t;
            do
            {
                t = NextToken();
                if (t.Type != TokenType.COMMENT)
                {
                    tokens.Add(t);
                }
            } while (t.Type != TokenType.EOF);

            return tokens;
        }

        private char Current => _position < _input.Length ? _input[_position] : '\0';
        private char Peek => _position + 1 < _input.Length ? _input[_position + 1] : '\0';

        private void Advance()
        {
            if (Current == '\n')
            {
                _line++;
                _column = 0;
            }
            _position++;
            _column++;
        }

        private Token NextToken()
        {
            SkipWhitespace();

            if (Current == '\0') return MakeToken(TokenType.EOF, "");

            // Comments
            if (Current == '/' && Peek == '/')
            {
                return LexComment();
            }

            // Identifiers / Keywords
            if (char.IsLetter(Current) || Current == '_')
            {
                return LexIdentifierOrKeyword();
            }

            // Numbers
            if (char.IsDigit(Current) || (Current == '-' && char.IsDigit(Peek)))
            {
                return LexNumber();
            }

            // Strings
            if (Current == '"')
            {
                return LexString();
            }

            // Operators & Symbols
            char c = Current;
            int startCol = _column;
            Advance();

            switch (c)
            {
                case '=':
                    if (Current == '=') { Advance(); return MakeToken(TokenType.EQUAL_EQUAL, "==", startCol); }
                    return MakeToken(TokenType.EQUALS, "=", startCol);
                case '<':
                    if (Current == '=') { Advance(); return MakeToken(TokenType.LESS_EQUAL, "<=", startCol); }
                    return MakeToken(TokenType.LESS_THAN, "<", startCol);
                case '>':
                    if (Current == '=') { Advance(); return MakeToken(TokenType.GREATER_EQUAL, ">=", startCol); }
                    return MakeToken(TokenType.GREATER_THAN, ">", startCol);
                case '!':
                    if (Current == '=') { Advance(); return MakeToken(TokenType.NOT_EQUAL, "!=", startCol); }
                    return MakeToken(TokenType.NOT, "!", startCol);
                case '&':
                    if (Current == '&') { Advance(); return MakeToken(TokenType.AND, "&&", startCol); }
                    break;
                case '|':
                    if (Current == '|') { Advance(); return MakeToken(TokenType.OR, "||", startCol); }
                    break;
                case '+': return MakeToken(TokenType.PLUS, "+", startCol);
                case '-': return MakeToken(TokenType.MINUS, "-", startCol);
                case '*': return MakeToken(TokenType.MULTIPLY, "*", startCol);
                case '/': return MakeToken(TokenType.DIVIDE, "/", startCol);
                case '(': return MakeToken(TokenType.LPAREN, "(", startCol);
                case ')': return MakeToken(TokenType.RPAREN, ")", startCol);
                case '{': return MakeToken(TokenType.LBRACE, "{", startCol);
                case '}': return MakeToken(TokenType.RBRACE, "}", startCol);
                case '[': return MakeToken(TokenType.LBRACKET, "[", startCol);
                case ']': return MakeToken(TokenType.RBRACKET, "]", startCol);
                case ',': return MakeToken(TokenType.COMMA, ",", startCol);
                case ';': return MakeToken(TokenType.SEMICOLON, ";", startCol);
                case ':': return MakeToken(TokenType.COLON, ":", startCol);
                case '.': return MakeToken(TokenType.DOT, ".", startCol);
            }

            return MakeToken(TokenType.UNKNOWN, c.ToString(), startCol);
        }

        private Token LexComment()
        {
            int startCol = _column;
            string value = "";
            while (Current != '\0' && Current != '\n')
            {
                value += Current;
                Advance();
            }
            return MakeToken(TokenType.COMMENT, value, startCol);
        }

        private Token LexIdentifierOrKeyword()
        {
            int startCol = _column;
            string value = "";
            while (char.IsLetterOrDigit(Current) || Current == '_')
            {
                value += Current;
                Advance();
            }

            if (Keywords.TryGetValue(value, out var type))
            {
                return MakeToken(type, value, startCol);
            }

            return MakeToken(TokenType.IDENTIFIER, value, startCol);
        }

        private Token LexNumber()
        {
            int startCol = _column;
            string value = "";
            
            if (Current == '-')
            {
                value += Current;
                Advance();
            }

            while (char.IsDigit(Current) || Current == '.')
            {
                value += Current;
                Advance();
            }

            return MakeToken(TokenType.NUMBER, value, startCol);
        }
        
        private Token LexString()
        {
            int startCol = _column;
            Advance(); // Skip open quote
            string value = "";
            
            while (Current != '\0' && Current != '"')
            {
                value += Current;
                Advance();
            }
            
            if (Current == '"') Advance(); // Skip close quote
            
            return MakeToken(TokenType.STRING, value, startCol);
        }

        private void SkipWhitespace()
        {
            while (char.IsWhiteSpace(Current))
            {
                Advance();
            }
        }

        private Token MakeToken(TokenType type, string value, int? colOverride = null)
        {
            return new Token(type, value, _line, colOverride ?? _column);
        }
    }
}
