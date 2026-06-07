# Mistakes Log

A record of debugging mistakes made during development, kept so future sessions don't repeat them.

---

## 2026-05-26 — Plot width not filling right-panel containers

**Symptom:** Profile/zoom plots had a right-side white gap. Opening DevTools (F12) fixed it instantly.

**Root cause:** Right-panel plots are created while their tab content divs are `display:none`. At that moment `clientWidth = 0`, so uPlot falls back to its 400 px default. No resize is triggered when the tab later becomes visible.

**What I missed:** The F12 clue was definitive — it triggered a window resize event, which called `resizePlots`, which worked correctly. The problem was not *how* to resize but *when*: at the moment the tab switches to visible, not at page load.

**Wrong approaches tried:**
1. `ResizeObserver` on `.plot` — `display:none` elements never emit ResizeObserver notifications.
2. Replaced all observers with `window.addEventListener("resize", resizePlots)` — only fires on browser window resize, not on tab visibility change.
3. `requestAnimationFrame(resizePlots)` at IIFE bottom — containers are still `display:none` for non-active tabs at startup, so still zero-width.

**Correct fix (found by separate agent):**
- Call `requestAnimationFrame(resizePlots)` inside `switchTab()` after toggling visibility — `clientWidth` is correct at that point.
- `ResizeObserver` on `.panel-right` (always visible) handles the initial layout case.

**Lesson:** When "opening DevTools fixes it," the problem is that a resize event fires at the right moment. Ask: what is the *first moment* the container becomes visible and has real dimensions? That is where the resize call belongs — not at page load.
