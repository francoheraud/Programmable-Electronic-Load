Project-local copy of Arduino-ESP32 SD 2.0.0 from framework 3.20017.241212+sha.dcc1105b.
Retains upstream license headers. No global framework modification.
Change: explicit bytewise R3/R7 response assembly; before filesystem access, require
CRC-checked CSD structure plus two ready OCR responses agreeing on capacity.
Corrects stored card type used by BOTH reads and writes; fails closed on disagreement.
Original startup misclassification mechanism is not yet proven; this validates and
corrects it before mounting rather than assuming all cards are SDHC.
