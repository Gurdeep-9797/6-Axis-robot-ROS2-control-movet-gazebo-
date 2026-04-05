import type { Config } from "tailwindcss";

export default {
  darkMode: ["class"],
  content: ["./pages/**/*.{ts,tsx}", "./components/**/*.{ts,tsx}", "./app/**/*.{ts,tsx}", "./src/**/*.{ts,tsx}"],
  prefix: "",
  theme: {
    container: {
      center: true,
      padding: "2rem",
      screens: { "2xl": "1400px" },
    },
    extend: {
      fontFamily: {
        mono: ["JetBrains Mono", "Consolas", "monospace"],
        sans: ["Segoe UI", "-apple-system", "sans-serif"],
      },
      colors: {
        border: "hsl(var(--border))",
        input: "hsl(var(--input))",
        ring: "hsl(var(--ring))",
        background: "hsl(var(--background))",
        foreground: "hsl(var(--foreground))",
        primary: { DEFAULT: "hsl(var(--primary))", foreground: "hsl(var(--primary-foreground))" },
        secondary: { DEFAULT: "hsl(var(--secondary))", foreground: "hsl(var(--secondary-foreground))" },
        destructive: { DEFAULT: "hsl(var(--destructive))", foreground: "hsl(var(--destructive-foreground))" },
        muted: { DEFAULT: "hsl(var(--muted))", foreground: "hsl(var(--muted-foreground))" },
        accent: { DEFAULT: "hsl(var(--accent))", foreground: "hsl(var(--accent-foreground))" },
        popover: { DEFAULT: "hsl(var(--popover))", foreground: "hsl(var(--popover-foreground))" },
        card: { DEFAULT: "hsl(var(--card))", foreground: "hsl(var(--card-foreground))" },
        sidebar: {
          DEFAULT: "hsl(var(--sidebar-background))",
          foreground: "hsl(var(--sidebar-foreground))",
          primary: "hsl(var(--sidebar-primary))",
          "primary-foreground": "hsl(var(--sidebar-primary-foreground))",
          accent: "hsl(var(--sidebar-accent))",
          "accent-foreground": "hsl(var(--sidebar-accent-foreground))",
          border: "hsl(var(--sidebar-border))",
          ring: "hsl(var(--sidebar-ring))",
        },
        status: {
          ok: "hsl(var(--status-ok))",
          warning: "hsl(var(--status-warning))",
          error: "hsl(var(--status-error))",
          info: "hsl(var(--status-info))",
        },
        viewport: {
          bg: "hsl(var(--viewport-bg))",
          grid: "hsl(var(--viewport-grid))",
        },
        ribbon: {
          bg: "hsl(var(--ribbon-bg))",
          border: "hsl(var(--ribbon-border))",
          hover: "hsl(var(--ribbon-hover))",
          active: "hsl(var(--ribbon-active))",
        },
        console: {
          bg: "hsl(var(--console-bg))",
          text: "hsl(var(--console-text))",
          error: "hsl(var(--console-error))",
          warning: "hsl(var(--console-warning))",
        },
        panel: {
          bg: "hsl(var(--panel-bg))",
          header: "hsl(var(--panel-header))",
          hover: "hsl(var(--panel-hover))",
        },
        code: {
          bg: "hsl(var(--code-bg))",
          keyword: "hsl(var(--code-keyword))",
          string: "hsl(var(--code-string))",
          comment: "hsl(var(--code-comment))",
          number: "hsl(var(--code-number))",
          function: "hsl(var(--code-function))",
        },
        block: {
          move: "hsl(var(--block-move))",
          logic: "hsl(var(--block-logic))",
          io: "hsl(var(--block-io))",
          wait: "hsl(var(--block-wait))",
          loop: "hsl(var(--block-loop))",
        },
      },
      borderRadius: {
        lg: "var(--radius)",
        md: "calc(var(--radius) - 2px)",
        sm: "calc(var(--radius) - 4px)",
      },
      keyframes: {
        "accordion-down": { from: { height: "0" }, to: { height: "var(--radix-accordion-content-height)" } },
        "accordion-up": { from: { height: "var(--radix-accordion-content-height)" }, to: { height: "0" } },
        "pulse-led": { "0%, 100%": { opacity: "1" }, "50%": { opacity: "0.5" } },
        "slide-in-right": { from: { transform: "translateX(100%)" }, to: { transform: "translateX(0)" } },
      },
      animation: {
        "accordion-down": "accordion-down 0.2s ease-out",
        "accordion-up": "accordion-up 0.2s ease-out",
        "pulse-led": "pulse-led 2s ease-in-out infinite",
        "slide-in-right": "slide-in-right 0.2s ease-out",
      },
    },
  },
  plugins: [require("tailwindcss-animate")],
} satisfies Config;
