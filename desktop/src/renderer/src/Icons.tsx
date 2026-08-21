import type { ReactElement, ReactNode, SVGProps } from 'react'

type IconName =
  | 'takeoff'
  | 'land'
  | 'hover'
  | 'emergency'
  | 'restart'
  | 'link'
  | 'video'
  | 'activity'

const paths: Record<IconName, ReactNode> = {
  takeoff: <><path d="m5 12 7-7 7 7" /><path d="M12 19V5" /></>,
  land: <><path d="M12 5v14" /><path d="m19 12-7 7-7-7" /></>,
  hover: <><circle cx="12" cy="12" r="9" /><path d="M9.5 9v6M14.5 9v6" /></>,
  emergency: <><path d="M10.3 2.9 1.8 17a2 2 0 0 0 1.7 3h17a2 2 0 0 0 1.7-3L13.7 2.9a2 2 0 0 0-3.4 0Z" /><path d="M12 9v4M12 17h.01" /></>,
  restart: <><path d="M20 11a8 8 0 1 0 1 4" /><path d="M20 4v7h-7" /></>,
  link: <><path d="M10 13a5 5 0 0 0 7.5.5l2-2a5 5 0 0 0-7-7l-1.1 1.1" /><path d="M14 11a5 5 0 0 0-7.5-.5l-2 2a5 5 0 0 0 7 7l1.1-1.1" /></>,
  video: <><rect x="3" y="5" width="14" height="14" rx="2" /><path d="m17 10 4-2v8l-4-2" /></>,
  activity: <path d="M3 12h4l2-7 4 14 2-7h6" />
}

export function Icon({ name, ...props }: { name: IconName } & SVGProps<SVGSVGElement>): ReactElement {
  return (
    <svg
      aria-hidden="true"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="1.8"
      strokeLinecap="round"
      strokeLinejoin="round"
      {...props}
    >
      {paths[name]}
    </svg>
  )
}
