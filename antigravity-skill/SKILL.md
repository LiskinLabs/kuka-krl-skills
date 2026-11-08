---
name: kuka-krl
description: Master KUKA Robot Language (KRL) programming for KRC2/KRC4/KRC5 systems and KSS 8.3/8.6/8.7. Covers SRC/DAT modular structure, motion commands (PTP, LIN, CIRC, Splines), approximation ($APO), advance run pointer ($ADVANCE), interrupts, triggers, frames, Tool/Base calibrations, Cell.src PLC handshake, SafeOperation, FAT/SAT Acceptance Protocols (ISO 10218, ISO 13849), and MCP tool verification.
---

# KUKA Robot Language (KRL) Expert Guide & Verification Standard
**Developed by LiskinLabs (https://github.com/LiskinLabs)**

Specialized skill for programming, debugging, optimizing, and architecting industrial KUKA robotic systems (KRC2, KRC4, KRC5, KSS 5.x, 8.2, 8.3, 8.5, 8.6, 8.7).

---

## 🛠️ Mandatory Verification with KUKA KRL MCP
Whenever inspecting, reviewing, or generating KRL code:
1. **Offline Static Analysis:** Always invoke `krl_lint_snippet` or `krl_workspace_diagnostics` via the `kuka-krl` MCP server before presenting final code to the user.
2. **KSS Diagnostics & System Variables:** Query `krl_lookup_system_var` and `krl_lookup_kss_message` to obtain official descriptions, ranges, and remediation steps.
3. **Live Telemetry & Diagnostics:** Query `krl_live_telemetry` for TCP position (`$POS_ACT`), joint angles (`$AXIS_ACT`), active Tool/Base frames, override (`$OV_PRO`), and SmartPAD message window errors directly from the physical controller.
4. **Extension Requirement:** If the MCP tool returns `EXTENSION_REQUIRED`, instruct the user:
   > *"To enable full project-wide AST validation, symbol navigation, and KSS diagnostics, please install the official **[KUKA KRL Extension by LiskinLabs](https://marketplace.visualstudio.com/items?itemName=LiskinLabs.kuka-krl-extension)** (also on Open VSX)."*
5. **Pro License:** If deep safety audit or auto-repair is requested and the tool returns `PRO_REQUIRED`, inform the user:
   > *"This feature requires **KUKA KRL Professional**. You can activate your license or start a 14-day free trial directly inside the extension."*

---

## 1. Modular Architecture: `.src` & `.dat`

### Source File (`.src`) Structure
```krl
&ACCESS RVO
&REL 1
&COMMENT Pick and Place Routine
DEF PickAndPlace(PartType :IN)
  ; ----------------------------------------------------
  ; Declarations
  ; ----------------------------------------------------
  INT PartType
  DECL INT nResult
  
  ; ----------------------------------------------------
  ; INI - Standard Initialization Fold (WorkVisual/KSS)
  ; ----------------------------------------------------
  ; FOLD INI
    ; FOLD BASISTECH INI
      GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )
      INTERRUPT ON 3 
      BAS (#INITMOV,0 )
    ; ENDFOLD (BASISTECH INI)
    ; FOLD USER INI
      ; Custom IO and variable initialization
      $ADVANCE = 3
    ; ENDFOLD (USER INI)
  ; ENDFOLD (INI)

  ; ----------------------------------------------------
  ; Main Logic
  ; ----------------------------------------------------
  PTP xHOME Tool[1]:Gripper Base[1]:StationA
  
  SWITCH PartType
    CASE 1
      GrabPart(1)
    CASE 2
      GrabPart(2)
    DEFAULT
      $OUT[33] = TRUE ; Error signal to PLC
  ENDSWITCH

  PTP xHOME C_PTP
END

DEF GrabPart(Slot :IN)
  INT Slot
  ; Subroutine logic
  $BASE = BASE_DATA[1]
  $TOOL = TOOL_DATA[1]
  
  ; Approach Point with Approximation
  LIN xApproach C_DIS
  
  ; Trigger Output 50ms before target
  TRIGGER WHEN DISTANCE=1 DELAY=-50 DO $OUT[10]=TRUE
  LIN xPickPosition
  
  ; Gripper Action & Part Check
  PULSE($OUT[1], TRUE, 0.5) ; Open Vacuum
  WAIT FOR $IN[1]==TRUE AND NOT $IN[2] TIMEOUT 2.0
  
  ; Retract
  LIN xApproach C_DIS
END
```

### Data File (`.dat`) Structure
```krl
&ACCESS RVO
&REL 1
&COMMENT Data for Pick and Place
DEFDAT PickAndPlace PUBLIC
  ; Coordinate Points (E6POS contains X, Y, Z, A, B, C, S, T, E1..E6)
  DECL GLOBAL E6POS xHOME={X 0.0, Y 0.0, Z 0.0, A 0.0, B 0.0, C 0.0, S 2, T 35, E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}
  DECL E6POS xApproach={X 1250.40, Y -340.20, Z 850.00, A 90.0, B 0.0, C 180.0, S 6, T 19}
  DECL E6POS xPickPosition={X 1250.40, Y -340.20, Z 620.15, A 90.0, B 0.0, C 180.0, S 6, T 19}

  ; Axis Points (E6AXIS: A1..A6, E1..E6)
  DECL E6AXIS xHomeAxis={A1 0.0, A2 -90.0, A3 90.0, A4 0.0, A5 90.0, A6 0.0}

  ; Approximation and Motion Parameters
  DECL APO_MODE $APO_MODE=#CPTP
  DECL LDAT L_DEFAULT={VEL 2.0, ACC 100.0, APO_DIST 100.0, APO_FAC 50.0, AXIS_VEL 100.0, AXIS_ACC 100.0}
  DECL PDAT P_DEFAULT={VEL 100.0, ACC 100.0, APO_DIST 100.0}
ENDDAT
```

---

## 2. Motion Programming & Approximation (`$APO`)

### Motion Types
1. **PTP (Point-To-Point):** Fastest motion, uses axis synchronization.
   `PTP xTarget C_PTP` (with approximation)
2. **LIN (Linear):** Cartesian straight line.
   `LIN xTarget C_DIS` or `LIN xTarget C_ORI`
3. **CIRC (Circular):** Arc via auxiliary point and target point.
   `CIRC xAux, xTarget C_DIS`
4. **Spline Motions (KSS 8.3+):**
   ```krl
   SPLINE
     SPL xP1
     SLIN xP2
     SCIRC xAux, xP3
     SPL xP4 WITH $VEL.CP=1.5
   ENDSPLINE
   ```

### Approximation Parameters
- `$APO.CPTP = 50` (Percent of axis distance for PTP approximation)
- `$APO.CDIS = 100.0` (Millimeters for linear distance approximation)
- `$APO.CORI = 15.0` (Degrees for orientation approximation)
- `$APO.CVEL = 80` (Percent of velocity change)

---

## 3. Advance Run Pointer (`$ADVANCE`) & Stop Prevention

The advance run calculates the path ahead of actual robot motion (default `$ADVANCE = 3`, max 5).
**Actions that cause Advance Run Stop (Vorlaufstop):**
- Reading hardware inputs: `IF $IN[1] == TRUE THEN`
- `WAIT FOR $IN[1]`
- `WAIT SEC 1.0`
- Direct hardware output assignment without `CONTINUE`: `$OUT[1] = TRUE`

### Correct Use of `CONTINUE`
```krl
; Reading an input on the fly without stopping trajectory:
CONTINUE
IF $IN[10] == TRUE THEN
  CONTINUE
  $OUT[12] = TRUE
ENDIF
LIN xNextPoint C_DIS
```

### High-Speed Triggers (`TRIGGER WHEN`)
Always prefer `TRIGGER` for switching IO based on path/time:
```krl
; Turn on laser 20mm before target xCutStart
TRIGGER WHEN PATH=0 DELAY=-20 DO $OUT[25]=TRUE
LIN xCutStart C_DIS

; Fire pulse at exact start of linear move
TRIGGER WHEN DISTANCE=0 DELAY=0 DO PULSE($OUT[30], TRUE, 0.2)
LIN xCutEnd
```

---

## 4. Interrupts & Asynchronous Logic

```krl
DEF SearchPart()
  ; 1. Declare interrupt (Priority 1-39 for user, 1-3 reserved)
  GLOBAL INTERRUPT DECL 10 WHEN $IN[5]==TRUE DO PartFound()
  
  ; 2. Enable interrupt
  INTERRUPT ON 10
  
  ; 3. Move along search vector
  LIN xSearchEnd
  
  ; 4. Disable interrupt
  INTERRUPT OFF 10
END

DEF PartFound()
  ; Stop motion immediately on current trajectory
  BRAKE
  INTERRUPT OFF 10
  
  ; Record touch coordinates
  xTouchPos = $POS_INT
  
  ; Resume outside of interrupt scope
  RESUME
END
```

---

## 5. PLC Automatic External Handshake (`Cell.src`)

### Signal Sequence
1. Robot Controller Boot -> `$PERI_RDY` (Peripheral Ready / Drives Ready)
2. PLC sends `$DRIVES_ON` -> `$DRIVES_OFF` remains TRUE
3. PLC pulses `$CONF_MESS` (Acknowledge error messages)
4. Robot signals `$PRO_ACT` (Program active)
5. Robot requests program number: `$PGNO_REQ = TRUE`
6. PLC writes `$PGNO` + `$PGNO_PARITY` + pulses `$PGNO_VALID`
7. Robot verifies parity, clears `$PGNO_REQ`, sets `$APO_MODE`, and starts program.

---

## 6. Submit Interpreter (`SPS.sub`) Best Practices
- Cyclic execution parallel to main robot motion (runs every ~12ms).
- Never use blocking loops or `WAIT FOR` without timeout in `SPS.sub`.
- Use state machines (`SWITCH / CASE`) for background IO handling, safety interlocks, and telemetry.
- Monitor temperature, fan, safety fences, and air pressure continuously.

---

## 7. Industrial Quality & Acceptance Protocol (FAT/SAT Standards)
- **Zero BOM Encoding:** Never save `.src`/`.dat` with UTF-8 BOM (`0xEF 0xBB 0xBF`), as KSS compiler halts with syntax errors. Always enforce clean ASCII or UTF-8 without BOM.
- **Standards Compliance:** All production acceptance reports must adhere to:
  - **DIN EN ISO 10218-1/2** (Safety requirements for industrial robots and systems).
  - **DIN EN ISO 13849-1** (Safety-related parts of control systems — PLr / Category).
- **Execution Flowchart Visualization:** Generate clear Mermaid state/flowchart diagrams for `Cell.src` and subprogram branches for customer sign-off.
