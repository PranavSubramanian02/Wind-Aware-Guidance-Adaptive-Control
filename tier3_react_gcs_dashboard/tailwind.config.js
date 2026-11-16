/** @type {import('tailwindcss').Config} */
export default {
  content: ["./index.html", "./src/**/*.{js,ts,jsx,tsx}"],
  theme: {
    extend: {
      fontFamily: {
        sans: [
          '"Segoe UI"',
          'Aptos',
          '-apple-system',
          'BlinkMacSystemFont',
          'Roboto',
          '"Helvetica Neue"',
          'Arial',
          'sans-serif',
        ],
        mono: [
          '"Segoe UI Mono"',
          'Consolas',
          '"SF Mono"',
          '"Cascadia Code"',
          'monospace',
        ],
      },
      colors: {
        cockpit: {
          bg: "#0d1117",
          panel: "#161b22",
          card: "#1c2128",
          border: "#30363d",
          borderSubtle: "#21262d",
          textPrimary: "#f0f6fc",
          textSecondary: "#8b949e",
          textMuted: "#6e7681",
          accentCyan: "#38bdf8",
          accentBlue: "#58a6ff",
          accentGreen: "#3fb950",
          accentAmber: "#d29922",
          accentRed: "#f85149",
        },
      },
    },
  },
  plugins: [],
};
