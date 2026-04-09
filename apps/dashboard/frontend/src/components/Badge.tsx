import { clsx } from "clsx";

const variants = {
  success: "bg-emerald-500/10 text-emerald-500 dark:bg-emerald-500/15 dark:text-emerald-400",
  danger: "bg-red-500/10 text-red-500 dark:bg-red-500/15 dark:text-red-400",
  warning: "bg-amber-500/10 text-amber-500 dark:bg-amber-500/15 dark:text-amber-400",
  info: "bg-blue-500/10 text-blue-500 dark:bg-blue-500/15 dark:text-blue-400",
  neutral: "bg-gray-500/10 text-gray-500 dark:bg-gray-500/15 dark:text-gray-400",
  purple: "bg-purple-500/10 text-purple-500 dark:bg-purple-500/15 dark:text-purple-400",
};

interface Props {
  variant?: keyof typeof variants;
  children: React.ReactNode;
  dot?: boolean;
  className?: string;
}

export default function Badge({ variant = "neutral", children, dot, className }: Props) {
  return (
    <span
      className={clsx(
        "inline-flex items-center gap-1.5 px-2 py-0.5 rounded-full text-xs font-medium",
        variants[variant],
        className
      )}
    >
      {dot && (
        <span
          className={clsx("w-1.5 h-1.5 rounded-full", {
            "bg-emerald-500": variant === "success",
            "bg-red-500": variant === "danger",
            "bg-amber-500": variant === "warning",
            "bg-blue-500": variant === "info",
            "bg-gray-500": variant === "neutral",
            "bg-purple-500": variant === "purple",
          })}
        />
      )}
      {children}
    </span>
  );
}
