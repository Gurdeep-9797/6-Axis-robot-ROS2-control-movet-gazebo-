namespace RoboForge_WPF.DSL
{
    public enum TokenType
    {
        // Keywords
        MOVEJ, MOVEL, WHILE, IF, ELSE, WAIT, SETIO, READIO, STOP, RETURN,
        
        // Identifiers & Values
        IDENTIFIER, NUMBER, STRING, BOOLEAN,
        
        // Operators
        EQUALS, PLUS, MINUS, MULTIPLY, DIVIDE, 
        EQUAL_EQUAL, NOT_EQUAL, LESS_THAN, GREATER_THAN, LESS_EQUAL, GREATER_EQUAL,
        AND, OR, NOT,
        
        // Symbols
        LPAREN, RPAREN, LBRACE, RBRACE, LBRACKET, RBRACKET, 
        COMMA, SEMICOLON, DOT, COLON,
        
        // Structural
        COMMENT, EOF, UNKNOWN
    }

    public class Token
    {
        public TokenType Type { get; }
        public string Value { get; }
        public int Line { get; }
        public int Column { get; }

        public Token(TokenType type, string value, int line, int column)
        {
            Type = type;
            Value = value;
            Line = line;
            Column = column;
        }

        public override string ToString()
        {
            return $"[{Line}:{Column}] {Type} '{Value}'";
        }
    }
}
