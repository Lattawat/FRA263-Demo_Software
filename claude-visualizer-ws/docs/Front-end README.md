# claude_visualizer — Frontend & WebSocket Guide

A beginner-friendly guide to how the web UI works: what HTML, CSS, and JavaScript each do,
how to write them, and how WebSocket connects the Python ROS 2 backend to the browser in real time.

---

## Table of Contents

1. [The three roles of HTML, CSS, and JavaScript](#1-the-three-roles-of-html-css-and-javascript)
2. [HTML — the structure](#2-html--the-structure)
   - [Document skeleton](#21-document-skeleton)
   - [Tags, elements, and nesting](#22-tags-elements-and-nesting)
   - [Attributes](#23-attributes)
   - [Common tags and what they do](#24-common-tags-and-what-they-do)
   - [Comments](#25-html-comments)
   - [How this project uses HTML](#26-how-this-project-uses-html)
3. [CSS — the appearance](#3-css--the-appearance)
   - [Basic rule syntax](#31-basic-rule-syntax)
   - [Selectors](#32-selectors)
   - [The box model](#33-the-box-model)
   - [Units and colors](#34-units-and-colors)
   - [Positioning](#35-positioning)
   - [Flexbox layout](#36-flexbox-layout)
   - [Grid layout](#37-grid-layout)
   - [CSS variables](#38-css-variables)
   - [Pseudo-classes and transitions](#39-pseudo-classes-and-transitions)
   - [Comments](#310-css-comments)
   - [How this project uses CSS](#311-how-this-project-uses-css)
4. [JavaScript — the behavior](#4-javascript--the-behavior)
   - [Variables: const, let, var](#41-variables-const-let-var)
   - [Data types](#42-data-types)
   - [Operators](#43-operators)
   - [Conditionals](#44-conditionals)
   - [Loops](#45-loops)
   - [Functions](#46-functions)
   - [Objects and arrays](#47-objects-and-arrays)
   - [The DOM API](#48-the-dom-api)
   - [Events and callbacks](#49-events-and-callbacks)
   - [Timers](#410-timers)
   - [Error handling](#411-error-handling)
   - [Modern syntax: template literals, destructuring, spread](#412-modern-syntax-template-literals-destructuring-spread)
   - [Comments](#413-js-comments)
   - [How this project uses JavaScript](#414-how-this-project-uses-javascript)
5. [WebSocket — real-time communication](#5-websocket--real-time-communication)
   - [Why not regular HTTP?](#51-why-not-regular-http)
   - [The handshake sequence](#52-the-handshake-sequence)
   - [Python side — broadcasting data](#53-python-side--broadcasting-data)
   - [JavaScript side — full lifecycle](#54-javascript-side--full-lifecycle)
   - [JSON: the shared language](#55-json-the-shared-language)
6. [How the three files load together](#6-how-the-three-files-load-together)
7. [Complete data flow diagram](#7-complete-data-flow-diagram)
8. [Key patterns quick reference](#8-key-patterns-quick-reference)

---

## 1. The three roles of HTML, CSS, and JavaScript

Think of building a webpage like building a room:

| File | Role | Analogy |
|------|------|---------|
| `index.html` | The **structure** — what exists | The walls, furniture, windows |
| `style.css`  | The **appearance** — how it looks | Paint, fabric, lighting |
| `app.js`     | The **behavior** — what it *does* | The electricity, motors, sensors |

The browser loads all three and combines them into the interactive page you see.
They share a common in-memory tree called the **DOM** (Document Object Model).
The browser parses HTML into the DOM. CSS reads the DOM to apply styles.
JavaScript reads and mutates the DOM to change what is visible and respond to the user.

```
index.html  ──►  DOM (live tree of elements in memory)
                   ▲              ▲
style.css reads it │  app.js reads│and mutates it
```

---

## 2. HTML — the structure

### 2.1 Document skeleton

Every HTML file starts with this boilerplate:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Page title shown in browser tab</title>

    <!-- Load CSS here -->
    <link rel="stylesheet" href="./style.css">
</head>
<body>
    <!-- All visible content goes here -->

    <!-- Load JavaScript here (at the bottom) -->
    <script src="./app.js"></script>
</body>
</html>
```

| Part | Meaning |
|------|---------|
| `<!DOCTYPE html>` | Tells the browser: "this is modern HTML5, not old HTML" |
| `<html lang="en">` | Root element that wraps everything; `lang` helps screen readers |
| `<head>` | Invisible metadata: title, CSS links, charset — nothing rendered |
| `<meta charset="UTF-8">` | Allows special characters (é, ü, ✓, …) to display correctly |
| `<meta name="viewport" ...>` | Makes the page scale correctly on phones and tablets |
| `<title>` | The text shown in the browser tab |
| `<body>` | Everything visible to the user lives here |

### 2.2 Tags, elements, and nesting

HTML uses **tags** — words wrapped in `< >`. Most tags come in pairs:

```html
<tagname>  content  </tagname>
opening tag ──────── closing tag (note the slash /)
```

An **element** is the whole unit: opening tag + content + closing tag.

```html
<h1>claude_visualizer</h1>
<p>This is a paragraph of text.</p>
<button>Click me</button>
```

Elements can be **nested** — placed inside other elements. The outer element is the **parent**; inner elements are **children**:

```html
<div>                    ← parent
    <h2>Live</h2>        ← child
    <div>                ← child (and also a parent to its children)
        <button>Pause</button>   ← grandchild
    </div>
</div>
```

**Indentation** (2 or 4 spaces) is not required by the browser but makes nesting visible to humans.

**Self-closing tags** have no content and no closing tag — they are complete by themselves:

```html
<br>          <!-- line break -->
<hr>          <!-- horizontal rule (divider line) -->
<img src="photo.png" alt="a photo">
<input type="text" placeholder="Type here">
<link rel="stylesheet" href="./style.css">
<meta charset="UTF-8">
```

### 2.3 Attributes

Attributes give extra information to a tag. They go inside the opening tag:

```html
<tagname attribute1="value1" attribute2="value2">content</tagname>
```

The two most important attributes for connecting HTML to CSS and JavaScript:

| Attribute | Rule | Used by |
|-----------|------|---------|
| `id="btn-pause"` | **Unique** — only one element per page can have this id | JavaScript: `getElementById("btn-pause")` |
| `class="dot connected"` | **Reusable** — many elements can share the same class; one element can have several classes separated by spaces | CSS: `.dot.connected { ... }` |

Other common attributes:

```html
<a href="https://example.com">Link text</a>   <!-- href: where to go -->
<img src="./photo.png" alt="description">      <!-- src: image path; alt: fallback text -->
<input type="number" placeholder="Enter value"> <!-- type: what kind of input -->
<button disabled>Can't click</button>           <!-- disabled: no value needed, just presence -->
```

### 2.4 Common tags and what they do

#### Structure / layout

| Tag | Purpose |
|-----|---------|
| `<div>` | Generic invisible container; the most-used tag for grouping |
| `<span>` | Inline container (doesn't break to a new line); used for styling a word inside text |
| `<header>` | Top section of the page (semantic; same as `<div>` but descriptive) |
| `<main>` | Main content area |
| `<footer>` | Bottom section |
| `<section>` | A thematic grouping of content |
| `<nav>` | Navigation links |

#### Text

| Tag | Result |
|-----|--------|
| `<h1>` … `<h6>` | Headings, largest to smallest |
| `<p>` | Paragraph |
| `<strong>` | **Bold** (also signals importance to screen readers) |
| `<em>` | *Italic* (emphasis) |
| `<br>` | Line break |

#### Interactive

| Tag | Purpose |
|-----|---------|
| `<button>` | Clickable button |
| `<input>` | Text field, number field, checkbox, etc. |
| `<a>` | Hyperlink |

#### In this project

```html
<!-- index.html — the status indicator -->
<span id="status-dot" class="dot disconnected"></span>
<span id="status-text">Disconnected</span>
```

`<span>` is like `<div>` but inline — it sits next to text without forcing a new line.
The dot starts with `class="dot disconnected"` (red). JavaScript changes it to `class="dot connected"` (green) when WebSocket connects.

### 2.5 HTML comments

```html
<!-- This is a comment. The browser ignores it. -->

<!-- Use comments to explain WHY, not what (the tag already says what) -->
<!-- LEFT PANEL: Live rolling plots -->
<section class="panel panel-left" id="panel-left">
```

### 2.6 How this project uses HTML

`index.html` defines the page in three zones:

```
<header>   ← title + Time Sync / Pos Sync buttons + status dot
<main>     ← two panels side by side (CSS Grid splits them 50/50)
    <section class="panel-left">   ← 3 live rolling plots
    <section class="panel-right">  ← Profile / Zoom tabs
<footer>   ← Clear, Pause, Crop, Start, Stop buttons + WS URL
```

Each plot is an **empty div** that JavaScript fills in at runtime:

```html
<div class="plot" id="plot-position"></div>
```

The `uPlot` library (loaded via `<script>`) injects a `<canvas>` element inside that div when `new uPlot(...)` is called in `app.js`.

---

## 3. CSS — the appearance

### 3.1 Basic rule syntax

```css
selector {
    property: value;
    property: value;
}
```

- **selector** — which element(s) to target
- **property** — what aspect to change
- **value** — what to set it to
- Each `property: value` pair ends with a semicolon `;`
- The whole block is wrapped in `{ }`

Example:

```css
button {
    background: #FF6C00;   /* orange background */
    color: white;           /* white text */
    padding: 7px 16px;      /* space inside: 7px top/bottom, 16px left/right */
    border-radius: 999px;   /* fully rounded ends */
    cursor: pointer;        /* show hand cursor when hovering */
}
```

### 3.2 Selectors

#### By type (tag name)

Targets every element of that type on the page:

```css
button { ... }   /* all <button> elements */
h1 { ... }       /* all <h1> elements */
```

#### By class (`.classname`)

```css
.controls { ... }        /* all elements with class="controls" */
.dot { ... }             /* all elements with class="dot" (or class="dot connected") */
```

#### By id (`#idname`)

```css
#btn-pause { ... }       /* the one element with id="btn-pause" */
```

#### Multiple classes together (`.a.b`)

```css
.dot.connected { background: green; }    /* element must have BOTH classes */
.dot.disconnected { background: red; }
```

#### Descendant (space between selectors)

```css
.panel-left .plot-container { ... }
/* targets .plot-container only when it is INSIDE .panel-left */
```

#### Direct child (`>`)

```css
.panel > h2 { ... }   /* targets <h2> that is a direct child of .panel */
```

#### Pseudo-classes (`:state`)

```css
button:hover  { background: #E85D1C; }  /* while mouse is over the button */
button:active { transform: translateY(1px); }  /* while being clicked */
.tab-btn.active { color: orange; }  /* when the .active class is present */
```

#### Specificity — who wins when rules conflict?

When two rules target the same element and the same property, the **more specific** rule wins:

```
id (#)  >  class (.)  >  type (tag)
```

```css
button       { color: white; }   /* least specific */
.tab-btn     { color: grey; }    /* more specific — wins over type */
#btn-special { color: red; }     /* most specific — wins over class */
```

When specificity is equal, the **rule that appears later** in the file wins.

`!important` overrides everything — use sparingly:

```css
.hidden { display: none !important; }   /* nothing can override this */
```

### 3.3 The box model

Every element is a rectangular box with four layers:

```
┌─────────────────────────────────────────┐
│               margin                    │  ← space OUTSIDE the element
│   ┌─────────────────────────────────┐   │
│   │            border               │   │  ← the visible border line
│   │   ┌─────────────────────────┐   │   │
│   │   │         padding         │   │   │  ← space INSIDE, between border and content
│   │   │   ┌─────────────────┐   │   │   │
│   │   │   │    content      │   │   │   │  ← the actual text / child elements
│   │   │   └─────────────────┘   │   │   │
│   │   └─────────────────────────┘   │   │
│   └─────────────────────────────────┘   │
└─────────────────────────────────────────┘
```

```css
button {
    margin:  10px;         /* push other elements away — space outside */
    padding: 7px 16px;     /* breathing room inside — between border and text */
    border:  2px solid orange;  /* width style color */
}
```

`box-sizing: border-box` (applied globally with `* { box-sizing: border-box; }` in this project) makes `width` and `height` include padding and border — otherwise you'd have to do mental arithmetic every time you set a width.

### 3.4 Units and colors

#### Size units

| Unit | Meaning | Best for |
|------|---------|----------|
| `px` | Pixels — absolute | Borders, small fixed sizes |
| `%` | Percentage of parent's size | Responsive widths |
| `vh` | 1% of the **viewport** (window) height | Full-height sections |
| `vw` | 1% of the viewport width | Full-width sections |
| `em` | Relative to the element's own font-size | Padding/margins that scale with text |
| `rem` | Relative to the root (`<html>`) font-size | Consistent spacing across the page |

#### Colors

```css
color: #FF6C00;          /* hex: #RRGGBB — most common */
color: #FF6C00CC;        /* hex with alpha: #RRGGBBAA */
color: rgb(255, 108, 0); /* red, green, blue (0–255) */
color: rgba(255, 108, 0, 0.5); /* same + alpha (0=transparent, 1=opaque) */
color: hsl(25, 100%, 50%);     /* hue (0–360°), saturation, lightness */
color: white;            /* named colors — 140 built-in names */
```

### 3.5 Positioning

By default all elements flow top-to-bottom. `position` changes that:

| Value | Behavior |
|-------|----------|
| `static` | Default — normal document flow |
| `relative` | Normal flow, but `top`/`left`/`right`/`bottom` nudge it relative to where it would have been |
| `absolute` | Removed from flow; positioned relative to the nearest `position: relative` ancestor |
| `fixed` | Removed from flow; positioned relative to the **viewport** — stays put when you scroll |

```css
/* The crop overlay sits on top of the plot */
.crop-overlay {
    position: absolute;   /* lifted out of flow */
    inset: 0;             /* shorthand for top:0; right:0; bottom:0; left:0 — fills parent */
    z-index: 5;           /* stacking order: higher number = in front */
}

/* The tooltip follows the cursor without scrolling */
.plot-tooltip {
    position: fixed;
    z-index: 200;
}
```

The **crop overlay** in this project needs `position: absolute` because it must sit exactly on top of the plot canvas. Its parent `.plot-wrapper` has `position: relative` so the overlay knows where "inset: 0" means.

### 3.6 Flexbox layout

`display: flex` turns an element into a **flex container**. Its direct children become **flex items** and are arranged automatically.

```css
/* Axis direction */
flex-direction: row;     /* items side by side (default) */
flex-direction: column;  /* items stacked vertically */

/* Main axis alignment (horizontal when row, vertical when column) */
justify-content: flex-start;     /* packed at the start */
justify-content: flex-end;       /* packed at the end */
justify-content: center;         /* centred */
justify-content: space-between;  /* first and last at edges, rest evenly spaced */
justify-content: space-around;   /* equal space around each item */

/* Cross axis alignment (perpendicular to flex-direction) */
align-items: stretch;    /* fill the cross axis (default) */
align-items: center;     /* centred on cross axis */
align-items: flex-start; /* align to start of cross axis */
```

On the **flex items** themselves:

```css
flex: 1;   /* grow to fill available space equally */
flex: 2;   /* grow twice as much as siblings with flex: 1 */
flex-shrink: 0;  /* refuse to shrink below its natural size */
```

In this project the header uses flexbox:

```css
header {
    display: flex;
    justify-content: space-between;  /* title left, controls right */
    align-items: center;             /* vertically centred */
}
```

The left panel stacks its three plots vertically, each taking equal height:

```css
.panel-left {
    display: flex;
    flex-direction: column;
}
.panel-left .plot-container {
    flex: 1;      /* each of the 3 plot containers takes 1/3 of the height */
    min-height: 0; /* allows flex children to shrink below their content size */
}
```

### 3.7 Grid layout

`display: grid` divides a container into rows and columns.

```css
main {
    display: grid;
    grid-template-columns: 1fr 1fr;  /* two equal columns */
    gap: 1px;                        /* space between cells (the thin divider line) */
}
```

`1fr` means "1 fraction of the available space". `1fr 1fr` = two equal columns.
You could write `1fr 2fr` for a 33%/66% split, or `300px 1fr` for a fixed sidebar and flexible content.

### 3.8 CSS variables

Defined in `:root` (the `<html>` element, highest-level scope), reused anywhere with `var(--name)`:

```css
:root {
    --accent:       #FF6C00;
    --accent-hover: #E85D1C;
    --bg:           #F7F7F7;
    --text:         #0A0A0A;
    --muted:        #6B6B6B;
    --border:       #E5E2DD;
    --connected:    #27AE60;
    --disconnected: #C0392B;
}

button { background: var(--accent); }
button:hover { background: var(--accent-hover); }
.dot.connected { background: var(--connected); }
```

To add a dark mode later you'd just redefine the variables — every element using them updates automatically.

### 3.9 Pseudo-classes and transitions

```css
/* State pseudo-classes */
button:hover  { background: var(--accent-hover); }   /* mouse over */
button:active { transform: translateY(1px); }         /* being clicked — gives a "press" feel */
.tab-btn.active { border-bottom-color: var(--accent); } /* JS adds .active class */

/* Smooth transition between states */
button {
    transition: background 120ms ease;  /* animate the background property over 120ms */
}
.tab-btn {
    transition: color 120ms ease, border-color 120ms ease; /* animate two properties */
}
```

`transition` makes style changes animate smoothly instead of snapping instantly.
Syntax: `transition: property duration timing-function`

### 3.10 CSS comments

```css
/* This is a CSS comment — the browser ignores it */

/* ── Section header ─────────── */  /* common style for visual separation */
```

### 3.11 How this project uses CSS

Key CSS decisions in `style.css`:

```css
/* 1. Global reset — consistent sizing everywhere */
* { box-sizing: border-box; }

/* 2. Full-height layout without scrollbars */
html, body { height: 100vh; overflow: hidden; }

/* 3. Body is a flex column: header | main | footer */
body { display: flex; flex-direction: column; }

/* 4. Main splits into two equal columns */
main { display: grid; grid-template-columns: 1fr 1fr; }

/* 5. Left panel: three plots fill equal vertical space */
.panel-left { display: flex; flex-direction: column; }
.panel-left .plot-container { flex: 1; }

/* 6. Connection status color via class swap */
.dot.connected    { background: var(--connected); }
.dot.disconnected { background: var(--disconnected); }

/* 7. .hidden utility — used everywhere by JavaScript */
.hidden { display: none !important; }
```

---

## 4. JavaScript — the behavior

JavaScript (JS) is the only programming language that runs natively in browsers.
It can read and modify the DOM, respond to user input, make network requests, and run logic.

### 4.1 Variables: `const`, `let`, `var`

A **variable** is a named container for a value.

```js
const PI = 3.14159;         // constant — cannot be reassigned
let   score = 0;            // block-scoped — can be reassigned
var   old = "avoid this";   // function-scoped — older, quirky; avoid in new code
```

| Keyword | Can reassign? | Scope |
|---------|--------------|-------|
| `const` | No | Block `{ }` |
| `let` | Yes | Block `{ }` |
| `var` | Yes | Function (ignores `{ }`) — confusing; avoid |

**Rule of thumb:** use `const` by default. Switch to `let` only when you need to reassign.

```js
// In app.js
const WS_URL = `ws://${location.hostname}:9090`;  // never changes — const
let ws = null;                                      // replaced on reconnect — let
```

### 4.2 Data types

```js
// Primitive types
const name    = "position";          // string — text in quotes (single, double, or backtick)
const value   = 3.14;               // number — integers and floats are the same type
const isOn    = true;               // boolean — true or false
const nothing = null;               // null — intentional absence of value
let   pending;                      // undefined — variable declared but not assigned

// Reference types
const point = { x: 1.0, y: 2.5 };  // object — key/value pairs
const times = [0.1, 0.2, 0.3];     // array — ordered list of values
```

Check the type of a value with `typeof`:

```js
typeof "hello"    // "string"
typeof 42         // "number"
typeof true       // "boolean"
typeof null       // "object"  ← historical quirk, not a real object
typeof undefined  // "undefined"
typeof {}         // "object"
typeof []         // "object"  ← arrays are objects; use Array.isArray([]) to check
```

### 4.3 Operators

#### Arithmetic

```js
5 + 3    // 8
5 - 3    // 2
5 * 3    // 15
5 / 3    // 1.666...
5 % 3    // 2  (remainder / modulo)
5 ** 2   // 25 (exponentiation)
```

#### Comparison

```js
5 === 5      // true  — strict equality (type AND value must match) — always use this
5 !== 3      // true  — strict not-equal
5 == "5"     // true  — loose equality (converts types) — avoid; surprising results
5 >  3       // true
5 >= 5       // true
5 <  3       // false
```

#### Logical

```js
true && false   // false  — AND: both must be true
true || false   // true   — OR:  at least one must be true
!true           // false  — NOT: flips the value
```

#### Assignment

```js
let x = 5;
x += 3;   // x = x + 3 → 8
x -= 2;   // x = x - 2 → 6
x *= 4;   // x = x * 4 → 24
x++;      // x = x + 1 → 25 (increment)
x--;      // x = x - 1 → 24 (decrement)
```

#### Nullish coalescing (`??`)

Returns the right side if the left side is `null` or `undefined`:

```js
const ref = state.timeRef ?? live.raw_time[0] ?? 0;
// Use state.timeRef if it's set, else first timestamp, else 0
```

This appears in `app.js`'s `displayRef()` function.

### 4.4 Conditionals

#### `if / else if / else`

```js
if (condition) {
    // runs when condition is true
} else if (otherCondition) {
    // runs when otherCondition is true
} else {
    // runs when none of the above matched
}
```

```js
// In app.js: pause / resume button
if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ command: "time_sync" }));
} else {
    const last = live.raw_time[live.raw_time.length - 1];
    if (last != null) state.timeRef = last;
}
```

#### Ternary operator (`condition ? a : b`)

A one-line `if/else` that produces a value:

```js
// Long form
if (state.paused) {
    ev.target.textContent = "Resume";
} else {
    ev.target.textContent = "Pause";
}

// Ternary — exactly the same result
ev.target.textContent = state.paused ? "Resume" : "Pause";
```

#### `switch`

Checks one value against many cases — cleaner than many `else if` chains:

```js
// In app.js — route WebSocket messages by topic
switch (msg.topic) {
    case "estimated_states":
        pushLive(msg);
        break;               // ← required! without break, execution falls into the next case
    case "actual_states":
        onActualStates(msg);
        break;
    case "event_trigger":
        onEventTrigger(msg);
        break;
    default:                 // optional — runs if no case matched
        console.warn("Unknown topic:", msg.topic);
}
```

### 4.5 Loops

#### `for` loop

```js
for (let i = 0; i < 5; i++) {
    console.log(i);   // prints 0, 1, 2, 3, 4
}
// structure: for (initialise; condition; step)
```

#### `while` loop

```js
while (live.raw_time.length && live.raw_time[0] < cutoff) {
    for (const arr of Object.values(live)) arr.shift();  // drop oldest sample
}
// Keeps running while the condition is true
```

#### `for...of` — iterate over an array's values

```js
for (const client of this._ws_clients) {    // Python equivalent: for client in clients
    await client.send(message);
}
```

#### `for...in` — iterate over an object's keys

```js
const data = { position: 1.2, velocity: 0.3 };
for (const key in data) {
    console.log(key, data[key]);   // "position" 1.2,  "velocity" 0.3
}
```

#### `forEach` — array method with a callback

```js
document.querySelectorAll(".tab-btn").forEach(btn => {
    btn.addEventListener("click", () => switchTab(btn.dataset.tab));
});
```

### 4.6 Functions

A function is a reusable block of code. You **define** it once and **call** it many times.

#### Function declaration

```js
function add(a, b) {
    return a + b;     // sends a value back to the caller
}

const result = add(3, 5);   // result = 8
```

#### Arrow function (modern, compact)

```js
// Named arrow function (stored in a const)
const add = (a, b) => a + b;      // single expression: no braces, no return needed

// Multi-line arrow function
const add = (a, b) => {
    const sum = a + b;
    return sum;
};

// No parameters
const greet = () => console.log("hello");

// One parameter — parentheses optional
const double = x => x * 2;
```

Arrow functions are used everywhere in `app.js`:

```js
// Callback passed to addEventListener
ws.addEventListener("open", () => {
    statusDot.className = "dot connected";
});

// Callback passed to forEach
overlayIds.map(id => document.getElementById(id));
```

#### Default parameter values

```js
function startProfile({ label = "", profile_id = "", expected_duration = 0, stamp = null } = {}) {
    // if label is not provided, it defaults to ""
}
```

#### Functions as values (callbacks)

In JavaScript, functions are values — you can pass them as arguments:

```js
setInterval(redraw, 33);
//          ^^^^^^  ── redraw is a function passed as a value (no parentheses!)
//                     setInterval will call redraw() every 33ms

// Compare:
setInterval(redraw(),  33);  // WRONG — calls redraw() once immediately, passes its return value
setInterval(redraw,    33);  // CORRECT — passes the function itself
```

### 4.7 Objects and arrays

#### Objects — key/value pairs

```js
const state = {
    paused:        false,
    profileActive: false,
    timeRef:       null,
};

// Read a property
console.log(state.paused);       // dot notation
console.log(state["paused"]);    // bracket notation (useful when key is dynamic)

// Write a property
state.paused = true;

// Add a new property
state.newThing = 42;

// Delete a property
delete state.newThing;

// Check if a key exists
"paused" in state     // true
state.missing         // undefined (not an error)
```

#### Object methods

```js
Object.keys(state)    // ["paused", "profileActive", "timeRef"]
Object.values(state)  // [false, false, null]
Object.entries(state) // [["paused",false], ["profileActive",false], ...]
```

```js
// In app.js — iterate over all live arrays to trim them
for (const arr of Object.values(live)) arr.shift();
```

#### Arrays — ordered lists

```js
const times = [];           // empty array
times.push(1.0);            // add to end:   [1.0]
times.push(1.1);            //               [1.0, 1.1]
times.shift();              // remove from front:  [1.1]
times.pop();                // remove from end:    []

times[0]                    // first element (index starts at 0)
times[times.length - 1]    // last element
times.length                // number of elements

// Common array methods
[1,2,3].map(x => x * 2)        // [2, 4, 6]   — transform each element
[1,2,3].filter(x => x > 1)    // [2, 3]      — keep only matching elements
[1,2,3].find(x => x > 1)      // 2           — first matching element
[1,2,3].some(x => x > 2)      // true        — is any element > 2?
[1,2,3].every(x => x > 0)     // true        — are all elements > 0?
[1,2,3].reduce((acc, x) => acc + x, 0)  // 6 — fold into single value
```

```js
// In app.js — build display time array
const tData = live.raw_time.map(t => t - ref);   // shift timestamps so t=0 is the reference
```

### 4.8 The DOM API

The DOM is the browser's live representation of the HTML page.
JavaScript manipulates it through the `document` object.

#### Finding elements

```js
document.getElementById("btn-pause")          // returns one element or null
document.querySelector(".tab-btn")            // returns first match (any CSS selector)
document.querySelectorAll(".tab-btn")         // returns all matches as a NodeList
element.querySelector(".plot-label")          // search within a specific element
element.closest(".plot-container")            // walk up the tree to the nearest match
```

#### Reading and writing content

```js
element.textContent = "Connected";    // set text (no HTML — safe against injection)
element.textContent                   // read text
element.innerHTML   = "<b>bold</b>";  // set HTML (careful: never use with user input)
```

#### Changing classes

```js
element.className = "dot connected";     // replace ALL classes
element.classList.add("hidden");         // add one class
element.classList.remove("hidden");      // remove one class
element.classList.toggle("active");      // add if missing, remove if present
element.classList.contains("hidden");    // returns true/false
```

#### Changing style directly

```js
element.style.left  = "120px";
element.style.width = "80px";
element.style.cssText = "";    // clear all inline styles at once
```

#### Reading data attributes

```html
<button class="tab-btn" data-tab="profile">PROFILE</button>
```

```js
btn.dataset.tab   // "profile"  — reads the data-tab attribute
```

In `app.js`, the tab switching uses this:

```js
document.querySelectorAll(".tab-btn").forEach(btn => {
    btn.addEventListener("click", () => switchTab(btn.dataset.tab));
});
```

### 4.9 Events and callbacks

An **event** is something that happens: a click, a key press, a message arriving, a timer firing.
`addEventListener(eventName, callback)` registers a function to run when the event occurs.

```js
element.addEventListener("click",      fn);   // mouse click
element.addEventListener("mousedown",  fn);   // mouse button pressed
element.addEventListener("mouseup",    fn);   // mouse button released
element.addEventListener("mousemove",  fn);   // mouse moves over element
element.addEventListener("mouseleave", fn);   // mouse leaves element
window.addEventListener("resize",      fn);   // browser window resized
window.addEventListener("mouseup",     fn);   // mouseup anywhere on page
ws.addEventListener("open",    fn);           // WebSocket connected
ws.addEventListener("close",   fn);           // WebSocket disconnected
ws.addEventListener("message", fn);           // WebSocket message received
```

The callback receives an **event object** with details:

```js
element.addEventListener("mousedown", e => {
    e.clientX          // mouse X position in the viewport
    e.clientY          // mouse Y position in the viewport
    e.target           // the element that was clicked
    e.preventDefault() // stop default browser behavior (e.g., link navigation)
});
```

### 4.10 Timers

```js
// Run once after a delay
setTimeout(callback, delayMs);

// Run repeatedly at an interval
const id = setInterval(callback, intervalMs);

// Stop a repeating timer
clearInterval(id);
```

```js
// In app.js
setInterval(redraw, 1000 / 30);  // redraw() runs ~30 times per second

ws.addEventListener("close", () => {
    setTimeout(connect, 1000);   // retry WebSocket connection after 1 second
});
```

### 4.11 Error handling

```js
try {
    msg = JSON.parse(ev.data);   // might throw if ev.data is not valid JSON
} catch (error) {
    console.error("Parse failed:", error);
    return;                      // bail out — don't process a broken message
}
```

```js
// In app.js — silently ignore bad messages
ws.addEventListener("message", ev => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch (_) { return; }
    // ... safe to use msg here
});
```

#### `console` debugging methods

```js
console.log("value:", x);      // general logging
console.warn("unexpected:", x); // yellow warning
console.error("failed:", e);    // red error
console.table(array);           // render array/object as a table
```

Open the browser's **Developer Tools** (`F12`) → **Console** tab to see all console output.

### 4.12 Modern syntax: template literals, destructuring, spread

#### Template literals (backtick strings)

```js
const port = 9090;
const url  = `ws://${location.hostname}:${port}`;
// Embeds expressions with ${ }. Much cleaner than "ws://" + hostname + ":" + port
```

#### Destructuring — unpack objects and arrays

```js
// Object destructuring
const { stamp, position, velocity, acceleration } = msg.data;
// Equivalent to:
// const stamp        = msg.data.stamp;
// const position     = msg.data.position;
// ...

// With default values
const { label = "", profile_id = "" } = msg.data;

// Array destructuring
const [first, second, ...rest] = [1, 2, 3, 4, 5];
// first=1, second=2, rest=[3,4,5]
```

```js
// In app.js
function pushLive(msg) {
    const { stamp, position, velocity, acceleration } = msg.data;
    live.raw_time.push(stamp);
    live.est_pos.push(position);
    // ...
}
```

#### Spread operator (`...`)

```js
// Copy an array
const copy = [...original];

// Copy an object
const newState = { ...state, paused: true };

// Spread into a function call
Math.max(...[1, 3, 2]);   // same as Math.max(1, 3, 2)
```

#### Optional chaining (`?.`)

Access a property without crashing if the object is `null` or `undefined`:

```js
u.data[1]?.[idx]    // safe: if u.data[1] is undefined, returns undefined instead of crashing
state?.posSync?.est  // safe chain
```

### 4.13 JS comments

```js
// Single-line comment — the rest of the line is ignored

/*
   Multi-line comment.
   Useful for longer explanations.
*/
```

### 4.14 How this project uses JavaScript

`app.js` has five responsibilities:

| Responsibility | Code location |
|---|---|
| Open and maintain the WebSocket connection | `connect()` function |
| Receive JSON messages and route them | `ws.addEventListener("message", ...)` |
| Accumulate data in rolling buffers | `pushLive()`, `onActualStates()`, `onEventTrigger()` |
| Redraw all charts at 30 Hz | `setInterval(redraw, 33)` |
| Handle user controls (buttons, mouse drag) | `addEventListener("click", ...)` blocks at the bottom |

The entire file is wrapped in an **IIFE** (Immediately Invoked Function Expression) to keep all variables private:

```js
(() => {
    "use strict";
    // everything here is scoped — invisible to other scripts
})();
```

`"use strict"` enables strict mode: common mistakes (undeclared variables, duplicate parameters) throw errors instead of silently doing the wrong thing.

---

## 5. WebSocket — real-time communication

### 5.1 Why not regular HTTP?

Normal HTTP is a one-shot request/response:

```
Browser:  "GET /data"  →  Server
Server:   "200 OK, here's data"  →  Browser
connection closes
```

For 100 Hz sensor data that model requires 100 requests per second — crushing overhead.

**WebSocket** upgrades an HTTP connection into a persistent, bidirectional channel:

```
Browser:  "Upgrade to WebSocket"  →  Server
Server:   "101 Switching Protocols — done"  →  Browser
connection stays open forever
Server can push any time:  → Browser
Browser can push any time: → Server
```

### 5.2 The handshake sequence

```
Browser (app.js)                        Python (web_visualizer.py)
        │                                          │
        │── new WebSocket("ws://host:9090") ──────►│
        │                                          │  _handle_client(websocket) fires
        │◄── cached last msg for each topic ───────│  (immediate snapshot)
        │                                          │
        │   ... ROS /estimated_states fires ...    │
        │                         _estimated_states_cb(msg)
        │                         _broadcast(estimated_states_to_json(msg))
        │◄── {"topic":"estimated_states","data":{…}}│
        │                                          │
        │   user clicks "Time Sync"                │
        │── {"command":"time_sync"} ──────────────►│
        │                         _handle_command() fires
        │◄── {"topic":"time_sync","data":{…}} ─────│
        │                                          │
        │   node restarts / network hiccup         │
        │   WebSocket "close" event fires          │
        │   setTimeout(connect, 1000)              │
        │── new WebSocket("ws://host:9090") ──────►│  (reconnect)
```

### 5.3 Python side — broadcasting data

**Step 1** — ROS message arrives, gets serialised to a dict:

```python
def estimated_states_to_json(msg: EncoderState) -> dict:
    return {
        "topic": "estimated_states",
        "data": {
            "stamp":        _stamp_to_sec(msg.header.stamp),
            "position":     msg.position,
            "velocity":     msg.velocity,
            "acceleration": msg.acceleration,
        },
    }
```

**Step 2** — `_broadcast` converts the dict to a JSON text string and pushes it to every browser:

```python
def _broadcast(self, payload: dict) -> None:
    message = json.dumps(payload)                       # dict  →  JSON string
    self._ws_last_messages[payload["topic"]] = message  # cache for new clients
    asyncio.run_coroutine_threadsafe(                   # cross-thread bridge
        self._broadcast_async(message), self._ws_loop_scheduler,
    )

async def _broadcast_async(self, message: str) -> None:
    for client in list(self._ws_clients):
        await client.send(message)                      # send to each browser
```

The `asyncio.run_coroutine_threadsafe` call is necessary because:
- ROS callbacks run on the **main thread**
- The WebSocket server runs on an **asyncio event loop in a separate thread**
- You cannot call async functions directly from a different thread — this is the safe bridge

**Step 3** — New clients get a snapshot immediately on connect:

```python
async def _handle_client(self, websocket) -> None:
    for cached in self._ws_last_messages.values():   # replay last msg per topic
        await websocket.send(cached)
    self._ws_clients.add(websocket)                  # now add to live broadcast set
    try:
        async for raw in websocket:                  # receive loop
            await self._handle_command(json.loads(raw))
    except websockets.ConnectionClosed:
        pass
    finally:
        self._ws_clients.discard(websocket)          # clean up on disconnect
```

**Step 4** — Receiving commands from the browser:

```python
async def _handle_command(self, cmd: dict) -> None:
    if cmd.get("command") == "time_sync":
        ref_sec = self.get_clock().now().nanoseconds / 1e9
        payload = json.dumps({"topic": "time_sync", "data": {"ref_stamp": ref_sec}})
        await self._broadcast_async(payload)
```

### 5.4 JavaScript side — full lifecycle

```js
const WS_URL = `ws://${location.hostname || "localhost"}:9090`;
let ws = null;

function connect() {
    ws = new WebSocket(WS_URL);      // 1. Open connection

    ws.addEventListener("open", () => {          // 2. Connected
        statusDot.className    = "dot connected";
        statusText.textContent = "Connected";
    });

    ws.addEventListener("close", () => {         // 3. Disconnected
        statusDot.className    = "dot disconnected";
        statusText.textContent = "Disconnected — retrying";
        ws = null;
        setTimeout(connect, 1000);              // retry after 1s
    });

    ws.addEventListener("error", () => { });    // 4. Error (close fires after this anyway)

    ws.addEventListener("message", ev => {       // 5. Incoming data
        let msg;
        try { msg = JSON.parse(ev.data); } catch (_) { return; }

        switch (msg.topic) {
            case "estimated_states": pushLive(msg);       break;
            case "actual_states":   onActualStates(msg); break;
            case "event_trigger":   onEventTrigger(msg); break;
            case "time_sync":       state.timeRef = msg.data.ref_stamp; break;
        }
    });
}
connect();

// 6. Sending data to Python
document.getElementById("btn-time-sync").addEventListener("click", () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ command: "time_sync" }));
    }
});
```

`ws.readyState` guards against sending on a closed socket:

| `readyState` | Value | Meaning |
|---|---|---|
| `WebSocket.CONNECTING` | 0 | Handshake in progress |
| `WebSocket.OPEN` | 1 | Connected and ready |
| `WebSocket.CLOSING` | 2 | Closing handshake in progress |
| `WebSocket.CLOSED` | 3 | Connection closed |

### 5.5 JSON: the shared language

JSON (JavaScript Object Notation) is a text format that both Python and JavaScript understand natively. It is the lingua franca of this project's WebSocket protocol.

```json
{
    "topic": "estimated_states",
    "data": {
        "stamp": 1716201234.567,
        "position": 1.2345,
        "velocity": 0.0031,
        "acceleration": -0.0002
    }
}
```

JSON rules:
- Keys must be **double-quoted strings**
- Values can be: string, number, boolean (`true`/`false`), `null`, array, or object
- No trailing commas
- No comments

| Language | Object → text | Text → object |
|---|---|---|
| Python | `json.dumps(dict)` | `json.loads(string)` |
| JavaScript | `JSON.stringify(obj)` | `JSON.parse(string)` |

---

## 6. How the three files load together

The browser reads HTML top-to-bottom. Order matters:

```html
<head>
    <!-- 1. CSS loads FIRST — page is never rendered unstyled -->
    <link rel="stylesheet" href="https://cdn.../uPlot.min.css">
    <link rel="stylesheet" href="./style.css">
</head>
<body>
    <!-- 2. HTML elements are defined -->
    <div id="plot-position"></div>
    <button id="btn-pause">Pause</button>

    <!-- 3. Scripts load LAST — all elements exist before JS runs -->
    <script src="https://cdn.../uPlot.iife.min.js"></script>
    <script src="./app.js"></script>
</body>
```

If `app.js` were at the top:

```js
const elPos = document.getElementById("plot-position");
// Returns null — <div id="plot-position"> doesn't exist yet!
new uPlot(opts, data, elPos);
// Crashes — cannot attach a chart to null
```

The **HTTP server** (port 8000) serves these files. It is a plain Python `http.server.HTTPServer` that maps URL paths to files in the `web/` directory:

```
Browser request:  GET http://localhost:8000/           → serves index.html
Browser request:  GET http://localhost:8000/style.css  → serves style.css
Browser request:  GET http://localhost:8000/app.js     → serves app.js
```

This is separate from the WebSocket server (port 9090), which only handles persistent WebSocket connections.

---

## 7. Complete data flow diagram

```
┌────────────────────────────────────────────────────────────────────────┐
│  Python: web_visualizer.py                                              │
│                                                                         │
│  ROS topic /estimated_states                                            │
│      │                                                                  │
│      ▼                                                                  │
│  _estimated_states_cb(msg: EncoderState)                                │
│      │                                                                  │
│      ├── push [pos, vel, acc] → LSL outlet (for robot_controller.py)   │
│      │                                                                  │
│      └── _broadcast(estimated_states_to_json(msg))                     │
│               │                                                         │
│               ├── json.dumps() converts dict → JSON text               │
│               ├── cache text in _ws_last_messages["estimated_states"]   │
│               └── asyncio.run_coroutine_threadsafe → _broadcast_async  │
│                           └── client.send(text) for each browser       │
└─────────────────────────────────┬──────────────────────────────────────┘
                                  │  WebSocket — persistent TCP on port 9090
                                  │  message format: plain JSON text
                                  ▼
┌────────────────────────────────────────────────────────────────────────┐
│  Browser: app.js                                                        │
│                                                                         │
│  ws.addEventListener("message", ev => {                                │
│      msg = JSON.parse(ev.data)          ← text → JS object             │
│      switch(msg.topic) {                                               │
│          case "estimated_states":                                      │
│              pushLive(msg)                                             │
│              ├── live.raw_time.push(stamp)                             │
│              ├── live.est_pos.push(position)                           │
│              └── trim rolling 10-second window                         │
│      }                                                                 │
│  })                                                                    │
│                                                                         │
│  setInterval(redraw, 33ms)    ← 30 Hz, decoupled from message rate     │
│      tData = live.raw_time.map(t => t - ref)                          │
│      plots.position.setData([tData, live.est_pos, live.act_pos])      │
│      uPlot redraws the <canvas> inside <div id="plot-position">       │
└─────────────────────────────────┬──────────────────────────────────────┘
                                  │  DOM — shared in-memory tree
                                  ▼
┌────────────────────────────────────────────────────────────────────────┐
│  index.html — defines the structure (what elements exist)               │
│  style.css  — defines the appearance (colors, sizes, layout)           │
└────────────────────────────────────────────────────────────────────────┘
```

### Three concurrent servers in one ROS node

`WebVisualizerNode.__init__` starts three servers using Python threads:

| Thread | What it does | Port |
|--------|-------------|------|
| ROS spin (main thread) | Receives ROS messages and runs subscription callbacks | — |
| `http-server` thread | Serves `index.html`, `app.js`, `style.css` via plain HTTP | 8000 |
| `ws-server` thread (asyncio loop) | Manages WebSocket connections; broadcasts JSON to all browsers | 9090 |

The ROS thread and the WebSocket asyncio loop run concurrently.
`asyncio.run_coroutine_threadsafe` is the bridge that lets the ROS thread safely schedule work on the asyncio loop.

---

## 8. Key patterns quick reference

### HTML

| Pattern | Example | Why |
|---------|---------|-----|
| `id=` for unique targets | `id="btn-pause"` | JS `getElementById` needs a unique handle |
| `class=` for styled groups | `class="dot connected"` | CSS and `querySelectorAll` can select many at once |
| `data-*` attributes | `data-tab="profile"` | Pass data to JS without extra DOM lookups |
| Scripts at bottom of `<body>` | `<script src="./app.js">` | DOM fully built before JS runs |
| Empty div as chart target | `<div id="plot-position"></div>` | uPlot injects `<canvas>` here at runtime |
| `class="hidden"` to start invisible | `<div id="profile-active" class="hidden">` | JS reveals it later |

### CSS

| Pattern | Example | Why |
|---------|---------|-----|
| CSS variables in `:root` | `--accent: #FF6C00` | Single source of truth for the design |
| `* { box-sizing: border-box }` | Global reset | `width` includes padding/border — no surprises |
| `flex: 1` to fill space | `.plot-container { flex: 1 }` | Three plots share the panel height equally |
| `position: absolute; inset: 0` | `.crop-overlay` | Fill the parent exactly, lifted out of flow |
| `position: fixed` | `.plot-tooltip` | Stays in place relative to the viewport |
| `.hidden { display: none !important }` | Utility class | `!important` so no other rule can accidentally un-hide |
| `transition: property ms ease` | `button { transition: background 120ms }` | Smooth hover/active animation |

### JavaScript

| Pattern | Example | Why |
|---------|---------|-----|
| `const` by default | `const WS_URL = ...` | Signals value never changes; prevents accidental reassignment |
| `===` not `==` | `msg.topic === "estimated_states"` | Strict equality; avoids type-coercion surprises |
| `??` for defaults | `state.timeRef ?? live.raw_time[0] ?? 0` | Cleaner than nested `if (x != null)` checks |
| Destructuring | `const { stamp, position } = msg.data` | Reads multiple keys in one line |
| Template literals | `` `ws://${location.hostname}:9090` `` | Embeds expressions cleanly in strings |
| IIFE wrapper | `(() => { "use strict"; ... })()` | Private scope; strict mode |
| `setInterval(fn, 33)` | Redraw loop | Decouples 100 Hz data from 30 Hz render |
| `try { JSON.parse } catch` | Message handler | Never crash on a bad network message |
| `classList.add/remove` | Show/hide elements | Lets CSS define the style; JS just toggles the class |

### WebSocket

| Pattern | Example | Why |
|---------|---------|-----|
| `new WebSocket(url)` | `app.js connect()` | Opens the persistent connection |
| `addEventListener("close", retry)` | `setTimeout(connect, 1000)` | Auto-recover from node restarts |
| `ws.readyState === WebSocket.OPEN` | Before `ws.send(...)` | Guard against sending on a closed socket |
| `JSON.stringify` before send | `ws.send(JSON.stringify({...}))` | WebSocket sends text; objects must be serialised |
| `JSON.parse(ev.data)` on receive | Message handler | Convert text back to a usable JS object |
| Cache last message per topic | `_ws_last_messages` dict | New browsers get an immediate snapshot |
| `asyncio.run_coroutine_threadsafe` | `_broadcast()` | Cross-thread bridge: ROS thread → asyncio loop |
