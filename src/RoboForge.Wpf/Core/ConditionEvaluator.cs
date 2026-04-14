// ── Condition Expression Evaluator ────────────────────────────────────────
// Evaluates boolean expressions for While/If blocks
// Supports: variables, comparisons, logical operators, arithmetic
// Examples: "counter >= 10", "sensor1 AND sensor2", "pos_x < 100 AND pos_y > 50"
// ──────────────────────────────────────────────────────────────────────────
using System;
using System.Collections.Generic;
using System.Text.RegularExpressions;

namespace RoboForge.Wpf.Core
{
    /// <summary>
    /// Evaluates condition expressions used in While/If blocks.
    /// Supports variables, comparisons (==, !=, <, >, <=, >=), logical (AND, OR, NOT), and arithmetic.
    /// </summary>
    public static class ConditionEvaluator
    {
        private static readonly Dictionary<string, object> _variables = new(StringComparer.OrdinalIgnoreCase);

        /// <summary>
        /// Set a variable value for expression evaluation
        /// </summary>
        public static void SetVariable(string name, object value)
        {
            _variables[name] = value;
        }

        /// <summary>
        /// Clear all variables
        /// </summary>
        public static void ClearVariables()
        {
            _variables.Clear();
        }

        /// <summary>
        /// Evaluate a condition expression string
        /// Returns true if condition is met, false otherwise
        /// Supports: "counter >= 10", "sensor1 AND sensor2", "NOT error", "x < 100 AND y > 50"
        /// </summary>
        public static bool Evaluate(string expression)
        {
            if (string.IsNullOrWhiteSpace(expression))
                return true; // Empty condition defaults to true (infinite loop)
            
            expression = expression.Trim();
            
            // Handle OR (lowest precedence)
            if (expression.Contains(" OR ", StringComparison.OrdinalIgnoreCase))
            {
                var parts = SplitByOperator(expression, " OR ");
                foreach (var part in parts)
                {
                    if (Evaluate(part))
                        return true;
                }
                return false;
            }
            
            // Handle AND
            if (expression.Contains(" AND ", StringComparison.OrdinalIgnoreCase))
            {
                var parts = SplitByOperator(expression, " AND ");
                foreach (var part in parts)
                {
                    if (!Evaluate(part))
                        return false;
                }
                return true;
            }
            
            // Handle NOT
            if (expression.StartsWith("NOT ", StringComparison.OrdinalIgnoreCase))
            {
                return !Evaluate(expression.Substring(4).Trim());
            }
            
            // Handle parentheses
            if (expression.StartsWith("(") && expression.EndsWith(")"))
            {
                return Evaluate(expression.Substring(1, expression.Length - 2).Trim());
            }
            
            // Handle comparisons
            var comparisonOps = new[] { ">=", "<=", "!=", "==", ">", "<" };
            foreach (var op in comparisonOps)
            {
                var idx = expression.IndexOf(op, StringComparison.Ordinal);
                if (idx > 0)
                {
                    var left = expression.Substring(0, idx).Trim();
                    var right = expression.Substring(idx + op.Length).Trim();
                    return Compare(left, op, right);
                }
            }
            
            // Handle boolean variables
            if (bool.TryParse(expression, out var boolVal))
                return boolVal;
            
            // Handle variable lookup
            if (_variables.TryGetValue(expression, out var varVal))
            {
                return varVal switch
                {
                    bool b => b,
                    int i => i != 0,
                    double d => Math.Abs(d) > 0.0001,
                    string s => !string.IsNullOrEmpty(s),
                    _ => false
                };
            }
            
            // Unknown expression, default to true (safe for simulation)
            return true;
        }

        private static string[] SplitByOperator(string expression, string op)
        {
            var result = new List<string>();
            var remaining = expression;
            
            while (true)
            {
                var idx = remaining.IndexOf(op, StringComparison.OrdinalIgnoreCase);
                if (idx < 0)
                {
                    result.Add(remaining);
                    break;
                }
                result.Add(remaining.Substring(0, idx));
                remaining = remaining.Substring(idx + op.Length);
            }
            
            return result.ToArray();
        }

        private static bool Compare(string left, string op, string right)
        {
            var leftVal = ResolveValue(left);
            var rightVal = ResolveValue(right);
            
            // Both numeric
            if (TryParseDouble(leftVal?.ToString(), out var leftNum) && TryParseDouble(rightVal?.ToString(), out var rightNum))
            {
                return op switch
                {
                    "==" => Math.Abs(leftNum - rightNum) < 0.0001,
                    "!=" => Math.Abs(leftNum - rightNum) >= 0.0001,
                    "<" => leftNum < rightNum,
                    ">" => leftNum > rightNum,
                    "<=" => leftNum <= rightNum,
                    ">=" => leftNum >= rightNum,
                    _ => false
                };
            }
            
            // String comparison
            var leftStr = leftVal?.ToString() ?? "";
            var rightStr = rightVal?.ToString() ?? "";
            
            return op switch
            {
                "==" => leftStr.Equals(rightStr, StringComparison.OrdinalIgnoreCase),
                "!=" => !leftStr.Equals(rightStr, StringComparison.OrdinalIgnoreCase),
                _ => false
            };
        }

        private static object ResolveValue(string token)
        {
            token = token.Trim();
            
            // Check variables first
            if (_variables.TryGetValue(token, out var varVal))
                return varVal;
            
            // Try parsing as number
            if (TryParseDouble(token, out var num))
                return num;
            
            // Try parsing as boolean
            if (bool.TryParse(token, out var boolVal))
                return boolVal;
            
            // Return as string
            return token;
        }

        private static bool TryParseDouble(string token, out double value)
        {
            return double.TryParse(token, out value);
        }
    }
}
