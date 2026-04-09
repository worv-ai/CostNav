import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "CostNav Dashboard",
  description: "Mission control for CostNav navigation benchmark",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="en" className="dark" suppressHydrationWarning>
      <body className="min-h-screen antialiased">{children}</body>
    </html>
  );
}
