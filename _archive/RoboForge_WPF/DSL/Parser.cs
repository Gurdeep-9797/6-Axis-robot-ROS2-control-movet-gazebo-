using System;
using System.Collections.Generic;

namespace RoboForge_WPF.DSL
{
    public class Parser
    {
        private readonly List<Token> _tokens;
        private int _position;

        public Parser(List<Token> tokens)
        {
            _tokens = tokens;
        }

        private Token Current => _position < _tokens.Count ? _tokens[_position] : _tokens[^1];
        private Token Peek => _position + 1 < _tokens.Count ? _tokens[_position + 1] : _tokens[^1];

        private Token Advance()
        {
            if (Current.Type != TokenType.EOF) _position++;
            return _tokens[_position - 1];
        }

        private bool Match(TokenType type)
        {
            if (Current.Type == type)
            {
                Advance();
                return true;
            }
            return false;
        }

        private Token Consume(TokenType type, string message)
        {
            if (Current.Type == type) return Advance();
            throw new Exception($"[Parse Error] Line {Current.Line}: {message}. Found '{Current.Value}' but expected {type}");
        }

        public ProgramNode Parse()
        {
            var program = new ProgramNode();
            while (Current.Type != TokenType.EOF)
            {
                var stmt = ParseStatement();
                if (stmt != null) program.Statements.Add(stmt);
            }
            return program;
        }

        private AstNode? ParseStatement()
        {
            if (Match(TokenType.MOVEJ)) return ParseMoveJ();
            if (Match(TokenType.MOVEL)) return ParseMoveL();
            if (Match(TokenType.WAIT)) return ParseWait();
            if (Match(TokenType.SETIO)) return ParseSetIO();
            
            // TODO: Add while, if, expressions etc.
            
            Advance(); // Skip unknown for now to avoid infinite loops
            return null;
        }

        private MoveJNode ParseMoveJ()
        {
            Consume(TokenType.LPAREN, "Expected '(' after movej");
            string target = Consume(TokenType.STRING, "Expected target waypoint name").Value;
            Consume(TokenType.COMMA, "Expected ',' after waypoint name");
            double velocity = double.Parse(Consume(TokenType.NUMBER, "Expected velocity").Value);
            Consume(TokenType.COMMA, "Expected ',' after velocity");
            int zone = int.Parse(Consume(TokenType.NUMBER, "Expected zone value").Value);
            Consume(TokenType.RPAREN, "Expected ')' after zone");
            Consume(TokenType.SEMICOLON, "Expected ';' after statement");
            return new MoveJNode(target, velocity, zone);
        }

        private MoveLNode ParseMoveL()
        {
            Consume(TokenType.LPAREN, "Expected '(' after movel");
            string target = Consume(TokenType.STRING, "Expected target waypoint name").Value;
            Consume(TokenType.COMMA, "Expected ',' after waypoint name");
            double velocity = double.Parse(Consume(TokenType.NUMBER, "Expected velocity").Value);
            Consume(TokenType.COMMA, "Expected ',' after velocity");
            int zone = int.Parse(Consume(TokenType.NUMBER, "Expected zone value").Value);
            Consume(TokenType.RPAREN, "Expected ')' after zone");
            Consume(TokenType.SEMICOLON, "Expected ';' after statement");
            return new MoveLNode(target, velocity, zone);
        }

        private WaitNode ParseWait()
        {
            Consume(TokenType.LPAREN, "Expected '(' after wait");
            double duration = double.Parse(Consume(TokenType.NUMBER, "Expected duration in ms").Value);
            Consume(TokenType.RPAREN, "Expected ')' after duration");
            Consume(TokenType.SEMICOLON, "Expected ';' after statement");
            return new WaitNode(duration);
        }

        private SetIONode ParseSetIO()
        {
            Consume(TokenType.LPAREN, "Expected '(' after set_io");
            int pin = int.Parse(Consume(TokenType.NUMBER, "Expected IO pin number").Value);
            Consume(TokenType.COMMA, "Expected ',' after pin number");
            bool state = Consume(TokenType.BOOLEAN, "Expected boolean state").Value.ToLower() == "true";
            Consume(TokenType.RPAREN, "Expected ')' after state");
            Consume(TokenType.SEMICOLON, "Expected ';' after statement");
            return new SetIONode(pin, state);
        }
    }
}
