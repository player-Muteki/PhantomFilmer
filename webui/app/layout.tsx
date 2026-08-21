import type { Metadata } from 'next';
import './globals.css';

export const metadata: Metadata = {
  title: 'PhantomFilmer｜真机飞行控制台',
  description: 'PhantomFilmer 真机连接、内嵌视频与飞行状态监控界面',
};

export default function RootLayout({ children }: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="zh-CN">
      <body>{children}</body>
    </html>
  );
}
