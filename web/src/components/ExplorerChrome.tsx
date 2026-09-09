import type { ReactNode } from 'react'

/** Full-viewport canvas with overlay chrome that scrolls instead of clipping. */
export default function ExplorerChrome({
  canvas,
  header,
  left,
  right,
  footer,
}: {
  canvas: ReactNode
  header: ReactNode
  left?: ReactNode
  right?: ReactNode
  footer?: ReactNode
}) {
  return (
    <main className="relative h-svh min-h-0 overflow-hidden bg-orbit-bg">
      <div className="absolute inset-0 pt-[4.25rem]">{canvas}</div>
      <div className="pointer-events-none absolute inset-x-0 top-[4.25rem] bottom-0 z-20 overflow-y-auto overscroll-contain">
        <div className="mx-auto flex min-h-full w-full max-w-[1600px] flex-col gap-3 p-3 pb-8 md:px-5">
          <div className="pointer-events-auto shrink-0">{header}</div>
          <div className="flex min-h-0 flex-1 flex-col gap-3 md:flex-row md:items-start md:justify-between">
            {left ? (
              <div className="pointer-events-auto max-h-[min(48svh,24rem)] w-full min-w-0 overflow-y-auto md:max-h-[calc(100svh-16rem)] md:w-[13.5rem] md:shrink-0">
                {left}
              </div>
            ) : (
              <div className="hidden md:block md:w-[13.5rem]" />
            )}
            <div className="hidden min-h-[4rem] flex-1 md:block" aria-hidden />
            {right ? (
              <div className="pointer-events-auto max-h-[min(48svh,24rem)] w-full min-w-0 overflow-y-auto md:max-h-[calc(100svh-16rem)] md:w-[16.5rem] md:shrink-0">
                {right}
              </div>
            ) : (
              <div className="hidden md:block md:w-[16.5rem]" />
            )}
          </div>
          {footer ? <div className="pointer-events-auto mx-auto w-full max-w-4xl shrink-0">{footer}</div> : null}
        </div>
      </div>
    </main>
  )
}
