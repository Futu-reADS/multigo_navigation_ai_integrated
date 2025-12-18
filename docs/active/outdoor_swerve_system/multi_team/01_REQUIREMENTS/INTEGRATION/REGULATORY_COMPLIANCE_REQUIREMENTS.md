# Regulatory Compliance Requirements

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Requirements Specification
**Status:** Active
**Version:** 1.0
**Last Updated:** 2025-12-17
**Owner:** Integration Team (All Teams), Legal/Compliance Officer

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [GDPR (General Data Protection Regulation - EU)](#2-gdpr-general-data-protection-regulation---eu)
3. [PIPEDA (Personal Information Protection - Canada)](#3-pipeda-personal-information-protection---canada)
4. [HIPAA (Health Insurance Portability - US)](#4-hipaa-health-insurance-portability---us)
5. [CCPA (California Consumer Privacy Act - US)](#5-ccpa-california-consumer-privacy-act---us)
6. [ISO 13849 (Safety of Machinery)](#6-iso-13849-safety-of-machinery)
7. [IEC 62443 (Industrial Cybersecurity)](#7-iec-62443-industrial-cybersecurity)
8. [ISO 27001 (Information Security Management)](#8-iso-27001-information-security-management)
9. [ADA / Section 508 (Accessibility - US)](#9-ada--section-508-accessibility---us)
10. [FCC / CE (Electromagnetic Compatibility)](#10-fcc--ce-electromagnetic-compatibility)
11. [Compliance Matrix](#11-compliance-matrix)
12. [Compliance Auditing](#12-compliance-auditing)

---

## 1. Executive Summary

### 1.1 Purpose

This document specifies regulatory compliance requirements for the outdoor wheelchair transport robot fleet system. It ensures the system meets legal, safety, privacy, and security standards in all target markets.

### 1.2 Applicable Regulations

| Regulation | Applies If | Priority | Owner |
|------------|-----------|----------|-------|
| **GDPR** | Operating in EU or processing EU residents' data | CRITICAL | Legal/Privacy Officer |
| **PIPEDA** | Operating in Canada or processing Canadian residents' data | HIGH | Legal/Privacy Officer |
| **HIPAA** | Handling Protected Health Information (PHI) in US | HIGH | Legal/Compliance Officer |
| **CCPA** | Operating in California or processing California residents' data | HIGH | Legal/Privacy Officer |
| **ISO 13849** | Safety-critical autonomous vehicles | CRITICAL | Safety Engineer |
| **IEC 62443** | Industrial control systems with cybersecurity | HIGH | Security Architect |
| **ISO 27001** | Information security management (voluntary, certification) | MEDIUM | Security Architect |
| **ADA/Section 508** | US government contracts or public facilities | HIGH | UX Designer |
| **FCC Part 15** | Selling/operating electronic devices in US | CRITICAL | Hardware Engineer |
| **CE Mark** | Selling equipment in EU | CRITICAL | Hardware Engineer |

### 1.3 Compliance Approach

**Strategy:**
1. **Built-In Compliance:** Design for compliance from day one (not retrofit)
2. **Documentation:** Maintain records for audits (ROPA, consent logs, audit trails)
3. **Continuous Monitoring:** Regular compliance reviews (quarterly)
4. **Third-Party Audits:** Annual audits for ISO 13849, ISO 27001 (if certified)

---

## 2. GDPR (General Data Protection Regulation - EU)

### 2.1 Overview

**Regulation:** EU General Data Protection Regulation (GDPR)
**Effective Date:** May 25, 2018
**Penalties:** Up to €20 million or 4% of global annual turnover (whichever is higher)
**Applies To:** Processing personal data of EU residents (regardless of where company is located)

### 2.2 Key GDPR Principles

### REQ-GDPR-001 [CRITICAL]
**Lawfulness, Fairness, Transparency (GDPR Art. 5(1)(a))**
- System SHALL process personal data lawfully, fairly, and transparently
- Legal basis: Consent, contract, legal obligation, or legitimate interest
- Privacy notice SHALL clearly explain data processing

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-001

### REQ-GDPR-002 [CRITICAL]
**Purpose Limitation (GDPR Art. 5(1)(b))**
- Personal data SHALL be collected for specified, explicit, and legitimate purposes
- Data SHALL NOT be processed in a manner incompatible with original purpose

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-002, REQ-PRIV-044

### REQ-GDPR-003 [CRITICAL]
**Data Minimization (GDPR Art. 5(1)(c))**
- System SHALL collect only data adequate, relevant, and limited to what is necessary
- No excessive data collection (e.g., do not collect SSN if not needed)

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-003, REQ-PRIV-040

### REQ-GDPR-004 [CRITICAL]
**Accuracy (GDPR Art. 5(1)(d))**
- Personal data SHALL be accurate and kept up to date
- Inaccurate data must be erased or rectified without delay

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-004, REQ-PRIV-023

### REQ-GDPR-005 [CRITICAL]
**Storage Limitation (GDPR Art. 5(1)(e))**
- Personal data SHALL be kept only as long as necessary for the purposes
- Define retention periods for each data type (see PRIVACY_REQUIREMENTS.md Section 8)

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-005, REQ-PRIV-047

### REQ-GDPR-006 [CRITICAL]
**Integrity and Confidentiality (GDPR Art. 5(1)(f))**
- Personal data SHALL be processed securely
- Protect against unauthorized access, loss, destruction, or damage

**Implementation:** See SECURITY_REQUIREMENTS.md REQ-SEC-032 (encryption), REQ-SEC-038 (TLS)

### REQ-GDPR-007 [CRITICAL]
**Accountability (GDPR Art. 5(2))**
- Data controller SHALL be responsible for and able to demonstrate compliance
- Maintain records of processing activities (ROPA)

**Implementation:** See Section 2.5 below

### 2.3 Data Subject Rights

### REQ-GDPR-008 [CRITICAL]
**Right to Access (GDPR Art. 15)**
- System SHALL provide data subjects with access to their personal data within 30 days of request
- Provide data in structured, commonly used format (JSON or PDF)

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-021, API endpoint `/api/gdpr/access/:user_id`

### REQ-GDPR-009 [CRITICAL]
**Right to Rectification (GDPR Art. 16)**
- System SHALL allow data subjects to correct inaccurate personal data within 30 days of request

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-023

### REQ-GDPR-010 [CRITICAL]
**Right to Erasure ("Right to Be Forgotten") (GDPR Art. 17)**
- System SHALL delete personal data upon request (with exceptions)
- Exceptions: Legal obligation, legal claims, public interest

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-025, REQ-PRIV-026 (anonymization)

### REQ-GDPR-011 [HIGH]
**Right to Data Portability (GDPR Art. 20)**
- System SHALL export data subject's personal data in machine-readable format (JSON)
- Support direct transfer to another controller (if technically feasible)

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-029, API endpoint `/api/gdpr/export/:user_id`

### REQ-GDPR-012 [HIGH]
**Right to Object (GDPR Art. 21)**
- Data subject SHALL be able to object to processing based on legitimate interest
- System must stop processing unless compelling legitimate grounds

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-031

### REQ-GDPR-013 [MEDIUM]
**Right to Restrict Processing (GDPR Art. 18)**
- Data subject MAY request restriction of processing in certain cases
- System SHALL mark data as "restricted" (do not process, but do not delete)

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-033

### 2.4 Special Categories of Personal Data

### REQ-GDPR-014 [CRITICAL]
**Explicit Consent for Special Categories (GDPR Art. 9)**
- System SHALL obtain explicit consent for processing special categories of data:
  - Health information (medical conditions, allergies)
  - Biometric data (facial recognition, if used)
- Consent must be freely given, specific, informed, unambiguous

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-009, REQ-PRIV-034

### 2.5 Records of Processing Activities (ROPA)

### REQ-GDPR-015 [CRITICAL]
**Maintain ROPA (GDPR Art. 30)**
- Organization SHALL maintain Record of Processing Activities
- ROPA SHALL document:
  - Name and contact details of controller and DPO
  - Purposes of processing
  - Categories of data subjects and personal data
  - Recipients of personal data (third parties)
  - International transfers
  - Retention periods
  - Security measures

**Implementation:** Create `ROPA.md` document (see Appendix A for template)

### 2.6 Data Protection Impact Assessment (DPIA)

### REQ-GDPR-016 [CRITICAL]
**DPIA for High-Risk Processing (GDPR Art. 35)**
- Data Protection Impact Assessment (DPIA) required for:
  1. Systematic monitoring (location tracking, video surveillance)
  2. Large-scale processing of special categories (medical data)
  3. Automated decision-making with legal effects
- DPIA completed before processing begins

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-068, REQ-PRIV-069

### 2.7 Data Breach Notification

### REQ-GDPR-017 [CRITICAL]
**Breach Notification to Supervisory Authority (GDPR Art. 33)**
- If personal data breach occurs, notify supervisory authority within **72 hours**
- Breach definition: Unauthorized access, loss, destruction, or disclosure

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-092

### REQ-GDPR-018 [CRITICAL]
**Breach Notification to Data Subjects (GDPR Art. 34)**
- If breach poses "high risk" to data subjects' rights, notify affected individuals without undue delay
- High risk examples: Medical data leaked, financial data compromised

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-093

### 2.8 Data Protection Officer (DPO)

### REQ-GDPR-019 [HIGH]
**Appoint DPO (GDPR Art. 37)**
- If organization is public authority, or engages in large-scale monitoring, or large-scale special categories processing, appoint Data Protection Officer
- DPO responsibilities: Monitor compliance, advise on DPIAs, cooperate with supervisory authority
- DPO contact info published in Privacy Notice

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-100

---

## 3. PIPEDA (Personal Information Protection - Canada)

### 3.1 Overview

**Regulation:** Personal Information Protection and Electronic Documents Act (PIPEDA)
**Applies To:** Private sector organizations in Canada (or processing Canadians' data)
**Enforced By:** Office of the Privacy Commissioner of Canada (OPC)
**Penalties:** Up to CAD $100,000 per violation (rare, usually recommendations)

### 3.2 PIPEDA Principles

PIPEDA is based on **10 Fair Information Principles:**

### REQ-PIPEDA-001 [CRITICAL]
**Principle 1: Accountability**
- Organization is responsible for personal information under its control
- Designate individual(s) accountable for compliance

**Implementation:** Assign Privacy Officer, document in org chart

### REQ-PIPEDA-002 [CRITICAL]
**Principle 2: Identifying Purposes**
- Identify purposes for which personal information is collected (at or before collection)
- Purposes SHALL be documented and communicated to individuals

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-044 (purpose documentation)

### REQ-PIPEDA-003 [CRITICAL]
**Principle 3: Consent**
- Obtain consent for collection, use, or disclosure of personal information
- Consent can be express or implied (depending on sensitivity)
- Consent can be withdrawn (subject to legal/contractual restrictions)

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-034 (explicit consent for medical data), REQ-PRIV-037 (withdrawal)

### REQ-PIPEDA-004 [CRITICAL]
**Principle 4: Limiting Collection**
- Collect only information necessary for identified purposes
- Collect by fair and lawful means

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-040 (data minimization)

### REQ-PIPEDA-005 [CRITICAL]
**Principle 5: Limiting Use, Disclosure, and Retention**
- Personal information SHALL NOT be used or disclosed for purposes other than those for which it was collected (except with consent or as required by law)
- Retain personal information only as long as necessary

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-045 (no secondary use), REQ-PRIV-047 (retention periods)

### REQ-PIPEDA-006 [CRITICAL]
**Principle 6: Accuracy**
- Personal information SHALL be accurate, complete, and up-to-date
- Update personal information as necessary to fulfill identified purposes

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-023 (right to rectification)

### REQ-PIPEDA-007 [CRITICAL]
**Principle 7: Safeguards**
- Personal information SHALL be protected by security safeguards appropriate to sensitivity
- Safeguards: Physical, organizational, technical measures

**Implementation:** See SECURITY_REQUIREMENTS.md (152 requirements)

### REQ-PIPEDA-008 [CRITICAL]
**Principle 8: Openness**
- Organization SHALL make information about its personal information policies and practices readily available
- Provide Privacy Policy (accessible, clear language)

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-072 (privacy notice)

### REQ-PIPEDA-009 [CRITICAL]
**Principle 9: Individual Access**
- Upon request, individual SHALL be informed of existence, use, and disclosure of personal information
- Individual SHALL be able to challenge accuracy and completeness

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-021 (right to access)

### REQ-PIPEDA-010 [CRITICAL]
**Principle 10: Challenging Compliance**
- Individual SHALL be able to challenge organization's compliance with PIPEDA
- Organization SHALL have procedures to receive and respond to complaints

**Implementation:** Create complaint handling procedure (see Appendix B)

---

## 4. HIPAA (Health Insurance Portability - US)

### 4.1 Overview

**Regulation:** Health Insurance Portability and Accountability Act (HIPAA)
**Applies To:** Covered Entities (healthcare providers, health plans, clearinghouses) and Business Associates (service providers)
**Applies If:** Handling Protected Health Information (PHI) in US
**Penalties:** Up to $50,000 per violation, $1.5M per year for identical violations; criminal penalties (up to $250,000 fine, 10 years prison)

**Determination:** If system stores/transmits medical diagnoses, treatment plans, prescription info, medical record numbers → HIPAA applies
- **Our System:** If storing "medical notes" field (allergies, mobility restrictions) → Potentially HIPAA-covered
- **Recommendation:** Consult HIPAA attorney for definitive determination

### 4.2 HIPAA Privacy Rule

### REQ-HIPAA-001 [CRITICAL]
**Business Associate Agreement (BAA) (45 CFR § 164.502)**
- If system is Business Associate (processes PHI on behalf of Covered Entity), sign BAA
- BAA must specify: Permitted uses, safeguards, breach notification, termination

**Implementation:** Legal team drafts BAA with healthcare facility

### REQ-HIPAA-002 [CRITICAL]
**Minimum Necessary Standard (45 CFR § 164.502(b))**
- Access to PHI SHALL be limited to minimum necessary for job function
- Example: Operator sees only "mobility restriction: wheelchair", not full medical diagnosis

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-081 (minimum necessary standard)

### REQ-HIPAA-003 [HIGH]
**Patient Rights**
- Right to access PHI (within 30 days)
- Right to amend PHI
- Right to accounting of disclosures (who accessed PHI)

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-083

### 4.3 HIPAA Security Rule

### REQ-HIPAA-004 [CRITICAL]
**Administrative Safeguards (45 CFR § 164.308)**
- Designate Security Official
- Conduct risk assessment (annually)
- Implement workforce training (annual HIPAA training)
- Implement sanction policy (discipline for violations)

**Implementation:** Create HIPAA Security Policy document

### REQ-HIPAA-005 [CRITICAL]
**Physical Safeguards (45 CFR § 164.310)**
- Facility access controls (badge access, visitor log)
- Workstation security (lock screens, privacy screens)
- Device and media controls (encrypt laptops, secure disposal)

**Implementation:** See SECURITY_ARCHITECTURE.md Section 12 (physical security)

### REQ-HIPAA-006 [CRITICAL]
**Technical Safeguards (45 CFR § 164.312)**
- Access control (unique user IDs, automatic logoff)
- Audit controls (log all PHI access)
- Integrity controls (detect unauthorized PHI modification)
- Transmission security (encrypt PHI in transit - TLS 1.3)

**Implementation:**
- See SECURITY_REQUIREMENTS.md REQ-SEC-017 (RBAC), REQ-SEC-092 (audit logging)
- See SECURITY_REQUIREMENTS.md REQ-SEC-038 (TLS 1.3)

### 4.4 HIPAA Breach Notification Rule

### REQ-HIPAA-007 [CRITICAL]
**Breach Notification (45 CFR § 164.404-414)**
- If PHI breach occurs, notify:
  1. Affected individuals (within 60 days)
  2. HHS Secretary (if >500 individuals affected, immediate; if <500, annual report)
  3. Media (if >500 individuals in same state/jurisdiction)

**Implementation:** See PRIVACY_REQUIREMENTS.md REQ-PRIV-082 (HIPAA breach notification)

---

## 5. CCPA (California Consumer Privacy Act - US)

### 5.1 Overview

**Regulation:** California Consumer Privacy Act (CCPA)
**Effective Date:** January 1, 2020 (amended by CPRA in 2023)
**Applies To:** Businesses that collect California residents' personal information AND meet thresholds:
- Annual gross revenues >$25 million, OR
- Buy/sell personal information of >50,000 consumers, OR
- Derive >50% of annual revenues from selling personal information
**Penalties:** Up to $7,500 per intentional violation

### 5.2 CCPA Consumer Rights

### REQ-CCPA-001 [HIGH]
**Right to Know (CCPA § 1798.100)**
- Consumer has right to know what personal information is collected, used, shared, sold
- Provide upon request (within 45 days)

**Implementation:** Similar to GDPR right to access (see PRIVACY_REQUIREMENTS.md REQ-PRIV-021)

### REQ-CCPA-002 [HIGH]
**Right to Delete (CCPA § 1798.105)**
- Consumer has right to request deletion of personal information
- Exceptions: Legal obligation, fraud prevention, internal use

**Implementation:** Similar to GDPR right to erasure (see PRIVACY_REQUIREMENTS.md REQ-PRIV-025)

### REQ-CCPA-003 [HIGH]
**Right to Opt-Out of Sale (CCPA § 1798.120)**
- Consumer has right to opt-out of sale of personal information
- "Sale" broadly defined (sharing for monetary or other valuable consideration)

**Implementation:**
- See PRIVACY_REQUIREMENTS.md REQ-PRIV-088 (no sale of personal data)
- Provide "Do Not Sell My Personal Information" link on website

### REQ-CCPA-004 [HIGH]
**Right to Non-Discrimination (CCPA § 1798.125)**
- Business SHALL NOT discriminate against consumer for exercising CCPA rights
- Cannot deny goods/services, charge different prices, or provide different quality

**Implementation:** Ensure service unaffected by CCPA requests

---

## 6. ISO 13849 (Safety of Machinery)

### 6.1 Overview

**Standard:** ISO 13849-1:2015 (Safety of machinery - Safety-related parts of control systems)
**Applies To:** Safety-critical control systems (e.g., autonomous vehicles with passengers)
**Purpose:** Ensure safety functions perform reliably (prevent harm to people)
**Target:** **Category 3, Performance Level d (PLd)** or **Safety Integrity Level 2 (SIL 2)**

### 6.2 Safety Functions

### REQ-ISO13849-001 [CRITICAL]
**Emergency Stop (E-Stop)**
- Vehicle SHALL have hardware emergency stop button (mushroom-style, red)
- E-stop SHALL cut motor power immediately (<1 second)
- E-stop SHALL be Category 3 (dual-channel, monitored)

**Implementation:**
- Hardware: Dual-channel safety relay (e.g., Pilz PNOZ)
- Software: Monitor safety relay status, detect failures

### REQ-ISO13849-002 [CRITICAL]
**Obstacle Detection (Safety Function)**
- Obstacle detection SHALL prevent collisions (stop vehicle if obstacle detected)
- Redundant sensors: LiDAR (primary) + Camera (backup)
- If both sensors fail → E-stop triggered automatically

**Implementation:**
- Software: Sensor fusion (LiDAR + camera)
- Hardware: Watchdog timer (if sensors unresponsive >5 seconds → E-stop)

### REQ-ISO13849-003 [CRITICAL]
**Speed Limiting (Safety Function)**
- Software speed limit: 1.5 m/s with passenger, 2.0 m/s empty
- Hardware speed limit: 2.5 m/s (absolute maximum, enforced by motor controller)
- Software SHALL NOT exceed hardware limit

**Implementation:** Motor controller firmware enforces max speed

### REQ-ISO13849-004 [HIGH]
**Safe State (Fail-Safe)**
- On critical error (sensor failure, software crash), vehicle SHALL enter safe state:
  - Stop motion (engage brakes)
  - Disable autonomous mode
  - Display warning (vehicle inoperable)

**Implementation:** See ERROR_HANDLING_REQUIREMENTS.md REQ-ERR-001 (fail-safe principle)

### 6.3 Diagnostic Coverage (DC)

### REQ-ISO13849-005 [CRITICAL]
**Diagnostic Coverage ≥60% (Category 3 requirement)**
- Safety functions SHALL have diagnostic coverage ≥60%
- Diagnostics: Self-tests (sensor health checks, watchdog timer, redundancy checks)
- Diagnostic interval: <1 second (real-time monitoring)

**Implementation:**
- LiDAR health check every 100ms
- Camera health check every 100ms
- E-stop circuit monitored every 10ms

### 6.4 Mean Time to Dangerous Failure (MTTFd)

### REQ-ISO13849-006 [HIGH]
**MTTFd ≥30 years (for PLd)**
- Each channel of safety function SHALL have Mean Time to Dangerous Failure ≥30 years
- Use reliable components (e.g., industrial-grade safety relays)

**Implementation:** Select components with known MTTFd values (from manufacturer datasheets)

### 6.5 Validation and Verification

### REQ-ISO13849-007 [CRITICAL]
**Safety Function Testing**
- Safety functions SHALL be tested monthly (E-stop, obstacle detection, speed limiting)
- Test procedure documented (see Appendix C)
- Test results logged (for audit)

**Implementation:** Maintenance team performs monthly safety tests

### REQ-ISO13849-008 [HIGH]
**Third-Party Safety Assessment**
- Safety functions SHALL be assessed by qualified safety engineer or third-party certification body
- Assessment SHALL verify Category 3, PLd achieved

**Implementation:** Engage TÜV, UL, or similar certification body

---

## 7. IEC 62443 (Industrial Cybersecurity)

### 7.1 Overview

**Standard:** IEC 62443 (Industrial communication networks - Network and system security)
**Applies To:** Industrial control systems (ICS), including autonomous vehicles, robots
**Purpose:** Protect against cyber threats (prevent unauthorized access, malware, sabotage)
**Target Security Level:** **SL 2** (Protection against intentional violation using simple means)

### 7.2 Foundational Requirements (IEC 62443-4-2)

### REQ-IEC62443-001 [CRITICAL]
**FR 1: Identification and Authentication Control**
- All users and devices SHALL be uniquely identified
- Authentication: Multi-factor for Admin, password (bcrypt) for Operator
- See SECURITY_REQUIREMENTS.md REQ-SEC-001, REQ-SEC-002

### REQ-IEC62443-002 [CRITICAL]
**FR 2: Use Control (Authorization)**
- Least privilege: Users access only necessary resources (RBAC)
- See SECURITY_REQUIREMENTS.md REQ-SEC-017

### REQ-IEC62443-003 [CRITICAL]
**FR 3: System Integrity**
- Software SHALL be protected against unauthorized modification
- Code signing: Firmware updates signed with private key, verified before install
- See SECURITY_REQUIREMENTS.md REQ-SEC-139

### REQ-IEC62443-004 [CRITICAL]
**FR 4: Data Confidentiality**
- Data in transit SHALL be encrypted (TLS 1.3)
- Data at rest SHALL be encrypted (AES-256)
- See SECURITY_REQUIREMENTS.md REQ-SEC-031, REQ-SEC-038

### REQ-IEC62443-005 [CRITICAL]
**FR 5: Restricted Data Flow**
- Network segmentation: Separate vehicle network from TVM network
- Firewall between networks
- See SECURITY_ARCHITECTURE.md Section 5 (network security)

### REQ-IEC62443-006 [CRITICAL]
**FR 6: Timely Response to Events**
- Security events SHALL be logged and alerted (SIEM)
- Incident response plan (see SECURITY_ARCHITECTURE.md Section 11)

### REQ-IEC62443-007 [HIGH]
**FR 7: Resource Availability**
- System SHALL be protected against denial-of-service (DoS) attacks
- Rate limiting, DDoS protection (Cloudflare)
- See SECURITY_REQUIREMENTS.md REQ-SEC-069

---

## 8. ISO 27001 (Information Security Management)

### 8.1 Overview

**Standard:** ISO/IEC 27001:2013 (Information security management systems - Requirements)
**Applies To:** Organizations managing sensitive information (voluntary certification)
**Purpose:** Establish, implement, maintain, and continually improve Information Security Management System (ISMS)
**Certification:** Third-party audit by accredited certification body (e.g., BSI, SGS)

### 8.2 ISO 27001 Controls (Annex A)

**114 controls across 14 domains.** Key controls for our system:

### REQ-ISO27001-001 [HIGH]
**A.9.2.1: User Registration and De-Registration**
- Formal user registration and de-registration process
- See SECURITY_REQUIREMENTS.md REQ-SEC-019 (user lifecycle)

### REQ-ISO27001-002 [HIGH]
**A.9.4.2: Secure Log-On Procedures**
- Multi-factor authentication for privileged users
- See SECURITY_REQUIREMENTS.md REQ-SEC-001

### REQ-ISO27001-003 [HIGH]
**A.10.1.1: Policy on Use of Cryptographic Controls**
- Cryptographic controls: TLS 1.3, AES-256
- See SECURITY_REQUIREMENTS.md REQ-SEC-031, REQ-SEC-038

### REQ-ISO27001-004 [CRITICAL]
**A.12.4.1: Event Logging**
- Security events logged with timestamp, user, action, result
- See SECURITY_REQUIREMENTS.md REQ-SEC-092

### REQ-ISO27001-005 [CRITICAL]
**A.16.1.1: Responsibilities and Procedures for Incident Management**
- Incident response plan with roles, procedures
- See SECURITY_ARCHITECTURE.md Section 11 (incident response)

**Note:** Full ISO 27001 certification is optional (expensive, time-consuming). Consider for Phase 3 (after 1-2 years) if needed for enterprise customers.

---

## 9. ADA / Section 508 (Accessibility - US)

### 9.1 Overview

**Regulation:** Americans with Disabilities Act (ADA) + Section 508 (Rehabilitation Act)
**Applies To:** US federal agencies, recipients of federal funding, places of public accommodation
**Purpose:** Ensure equal access for people with disabilities
**Standard:** **WCAG 2.0 Level AA** (or WCAG 2.1 Level AA)

### 9.2 Accessibility Requirements

### REQ-ADA-001 [HIGH]
**WCAG 2.1 Level AA Compliance**
- TVM Dashboard SHALL comply with WCAG 2.1 Level AA
- See NONFUNCTIONAL_REQUIREMENTS.md REQ-USE-003

### REQ-ADA-002 [HIGH]
**Keyboard Accessibility**
- All interactive elements SHALL be keyboard-accessible (no mouse required)
- Tab order logical (top-to-bottom, left-to-right)

### REQ-ADA-003 [HIGH]
**Color Contrast**
- Color contrast ratio SHALL be ≥4.5:1 (normal text), ≥3:1 (large text)
- Do not rely solely on color to convey information (use icons, labels)

### REQ-ADA-004 [MEDIUM]
**Screen Reader Support**
- Dashboard SHALL be usable with screen readers (JAWS, NVDA, VoiceOver)
- All images SHALL have alt text
- Form fields SHALL have labels

### REQ-ADA-005 [MEDIUM]
**Captions and Transcripts**
- If dashboard includes video/audio content, provide captions and transcripts

**Implementation:** See NONFUNCTIONAL_REQUIREMENTS.md Section 6.2 (usability requirements)

---

## 10. FCC / CE (Electromagnetic Compatibility)

### 10.1 FCC (US)

**Regulation:** FCC Part 15 (Radio Frequency Devices)
**Applies To:** Electronic devices sold or operated in US
**Purpose:** Prevent harmful interference to radio communications

### REQ-FCC-001 [CRITICAL]
**FCC Part 15 Class B Compliance**
- Vehicle compute unit and electronics SHALL comply with FCC Part 15 Class B (residential environment)
- Radiated emissions limit: <100 µV/m at 3 meters (30-88 MHz)

**Implementation:**
- EMC testing by accredited lab (e.g., Intertek, UL)
- FCC Declaration of Conformity (DoC) or Certification

### REQ-FCC-002 [HIGH]
**FCC ID (If Transmitter)**
- If vehicle has WiFi/Bluetooth transmitter (>-1.0 dBm), obtain FCC ID
- FCC ID displayed on device label

**Implementation:** Submit FCC ID application (via TCB - Telecommunications Certification Body)

### 10.2 CE (EU)

**Regulation:** CE Marking (Conformité Européenne)
**Applies To:** Products sold in European Economic Area (EEA)
**Purpose:** Declare conformity with EU safety, health, environmental standards

### REQ-CE-001 [CRITICAL]
**CE Marking**
- Vehicle SHALL bear CE marking (if sold in EU)
- Declaration of Conformity (DoC) signed by manufacturer

**Implementation:**
- EMC Directive (2014/30/EU): EMC testing
- Low Voltage Directive (2014/35/EU): Electrical safety testing
- Machinery Directive (2006/42/EC): Safety of machinery (ISO 13849)

### REQ-CE-002 [HIGH]
**EU Technical File**
- Maintain Technical File with:
  - Product description
  - Design drawings
  - Risk assessment
  - Test reports (EMC, safety)
  - Declaration of Conformity

**Implementation:** Hardware team maintains Technical File

---

## 11. Compliance Matrix

### 11.1 Requirements to Standards Mapping

| Requirement Area | GDPR | PIPEDA | HIPAA | CCPA | ISO 13849 | IEC 62443 | ISO 27001 | ADA | FCC/CE |
|------------------|------|--------|-------|------|-----------|-----------|-----------|-----|--------|
| **Authentication (MFA)** | Art. 32 | Princ. 7 | §164.312 | - | - | FR 1 | A.9.4.2 | - | - |
| **Authorization (RBAC)** | Art. 25 | Princ. 7 | §164.312 | - | - | FR 2 | A.9.2.3 | - | - |
| **Encryption at Rest** | Art. 32 | Princ. 7 | §164.312 | - | - | FR 4 | A.10.1.1 | - | - |
| **Encryption in Transit** | Art. 32 | Princ. 7 | §164.312 | - | - | FR 4 | A.10.1.1 | - | - |
| **Audit Logging** | Art. 30 | Princ. 9 | §164.312 | §1798.100 | - | FR 6 | A.12.4.1 | - | - |
| **Data Retention** | Art. 5(1)(e) | Princ. 5 | - | - | - | - | A.11.2.7 | - | - |
| **Right to Access** | Art. 15 | Princ. 9 | §164.524 | §1798.100 | - | - | - | - | - |
| **Right to Erasure** | Art. 17 | - | - | §1798.105 | - | - | - | - | - |
| **Breach Notification** | Art. 33 | PIPEDA 10.1 | §164.404 | - | - | FR 6 | A.16.1.2 | - | - |
| **E-Stop (Safety)** | - | - | - | - | Cat 3 | - | - | - | - |
| **Obstacle Detection** | - | - | - | - | Cat 3 | - | - | - | - |
| **Network Segmentation** | Art. 32 | Princ. 7 | §164.312 | - | - | FR 5 | A.13.1.3 | - | - |
| **WCAG 2.1 AA** | - | - | - | - | - | - | - | §508 | - |
| **EMC Testing** | - | - | - | - | - | - | - | - | FCC 15, EMC Dir |

### 11.2 Compliance Status Dashboard

**Create Grafana dashboard to track compliance status:**

| Standard | Status | Last Audit | Next Audit | Findings |
|----------|--------|------------|------------|----------|
| **GDPR** | 🟢 Compliant | 2025-06-01 | 2026-06-01 | 0 open |
| **PIPEDA** | 🟢 Compliant | 2025-06-01 | 2026-06-01 | 0 open |
| **HIPAA** | 🟡 In Progress | N/A | 2026-03-01 | 3 open (low severity) |
| **ISO 13849** | 🟡 In Progress | N/A | 2026-04-01 | Safety assessment pending |
| **IEC 62443** | 🟢 Compliant | 2025-08-01 | 2026-08-01 | 0 open |
| **WCAG 2.1 AA** | 🟢 Compliant | 2025-09-01 | 2026-09-01 | 0 open |
| **FCC Part 15** | 🟢 Certified | 2025-05-01 | 2030-05-01 (5-year) | FCC ID: XYZ123 |

**Legend:**
- 🟢 Compliant: All requirements met, no open findings
- 🟡 In Progress: Implementation ongoing, some findings open
- 🔴 Non-Compliant: Critical findings, remediation required

---

## 12. Compliance Auditing

### 12.1 Audit Schedule

| Audit Type | Frequency | Conducted By | Purpose |
|------------|-----------|--------------|---------|
| **Internal Compliance Review** | Quarterly | Compliance Officer | Check compliance status, identify gaps |
| **GDPR Audit** | Annually | Internal or External Auditor | Verify GDPR compliance, update ROPA |
| **ISO 13849 Safety Assessment** | Annually | Qualified Safety Engineer or TÜV | Verify safety functions (Category 3, PLd) |
| **IEC 62443 Cybersecurity Audit** | Annually | Internal Security Team | Verify cybersecurity controls (SL 2) |
| **HIPAA Audit** | Annually (if applicable) | HIPAA Compliance Firm | Verify HIPAA Security Rule compliance |
| **Penetration Testing** | Annually | Third-Party Security Firm | Identify vulnerabilities |
| **WCAG Accessibility Audit** | Annually | Accessibility Consultant | Verify WCAG 2.1 AA compliance |

### 12.2 Audit Preparation

**Documents to Prepare:**
1. **ROPA (Record of Processing Activities)** - See Appendix A
2. **Privacy Policy** - Public-facing, clear language
3. **Security Policy** - Internal, technical details
4. **Incident Response Plan** - Procedures, contact list
5. **Data Breach Log** - All breaches (even if no notification required)
6. **Consent Records** - Proof of consent for special categories
7. **Audit Logs** - Sample logs showing PII access tracking
8. **Training Records** - Proof of annual privacy/security training
9. **Vendor Contracts** - Data Processing Agreements (DPAs) with vendors
10. **Technical Documentation** - Architecture diagrams, security controls

### 12.3 Continuous Compliance Monitoring

**Automated Checks (CI/CD Pipeline):**
- Vulnerability scanning (Snyk, Dependabot)
- Static analysis for security issues (Semgrep, SonarQube)
- License compliance (check for GPL violations)

**Manual Checks (Quarterly):**
- Review new features for privacy impact (DPIA if needed)
- Review vendor list (any new subprocessors?)
- Review audit logs for anomalies (unauthorized PII access?)

---

## Appendix A: ROPA Template

**Record of Processing Activities (GDPR Art. 30)**

```markdown
# Record of Processing Activities (ROPA)

## Organization Details
- **Name:** [Your Organization Name]
- **Address:** [Address]
- **Data Protection Officer (DPO):** [Name, Email, Phone]

## Processing Activity 1: Resident Transport Management

### 1. Purpose of Processing
- Provide wheelchair transport services for residents in healthcare facilities

### 2. Legal Basis
- **Contract:** Performance of contract with facility (service agreement)
- **Consent:** For processing special categories (medical data)

### 3. Categories of Data Subjects
- Residents (passengers)
- Healthcare facility staff (operators, nurses)

### 4. Categories of Personal Data
- **Identification data:** Name, date of birth, photo
- **Contact data:** Phone, emergency contact
- **Special categories:** Medical notes (allergies, mobility restrictions)
- **Location data:** Pickup/dropoff locations, traveled routes

### 5. Categories of Recipients
- Healthcare facility staff (operators, nurses) - need-to-know basis
- Cloud service providers (AWS) - Business Associate Agreement in place

### 6. International Transfers
- Data stored in AWS us-east-1 (US) and ca-central-1 (Canada)
- For EU residents: Standard Contractual Clauses (SCCs) with AWS

### 7. Retention Period
- Resident profiles: Active + 3 years after last service
- Mission logs: 3 years
- Telemetry: 90 days

### 8. Security Measures
- Encryption at rest (AES-256)
- Encryption in transit (TLS 1.3)
- Access control (RBAC)
- Audit logging (all PII access logged)

---

## Processing Activity 2: [Add more activities]

...
```

---

## Appendix B: PIPEDA Complaint Handling Procedure

**Procedure for Handling Privacy Complaints:**

1. **Receipt of Complaint:**
   - Complaints received via email (`privacy@example.com`) or web form
   - Acknowledge receipt within 1 business day

2. **Investigation:**
   - Privacy Officer investigates complaint (review logs, interview staff)
   - Investigation completed within 30 days

3. **Response:**
   - Provide written response to complainant (findings, remedial actions)
   - If complaint upheld: Implement corrective measures (e.g., delete data, update policy)

4. **Escalation:**
   - If complainant not satisfied, inform of right to contact Office of the Privacy Commissioner of Canada (OPC)

5. **Documentation:**
   - Maintain log of all complaints and resolutions (for audit)

---

## Appendix C: ISO 13849 Safety Function Test Procedure

**Monthly Safety Test Checklist:**

**Test 1: Emergency Stop (E-Stop)**
- [ ] Press E-stop button (red mushroom button)
- [ ] Verify: Vehicle stops immediately (<1 second)
- [ ] Verify: Motor power cut (wheels cannot be manually pushed)
- [ ] Verify: Display shows "E-STOP ACTIVATED"
- [ ] Release E-stop, restart vehicle
- [ ] Verify: Vehicle operational

**Test 2: Obstacle Detection**
- [ ] Place obstacle (e.g., cardboard box) in front of vehicle
- [ ] Command vehicle to drive forward
- [ ] Verify: Vehicle stops before obstacle (≥30 cm clearance)
- [ ] Verify: No collision
- [ ] Remove obstacle

**Test 3: Speed Limiting**
- [ ] Command vehicle to maximum speed (with passenger)
- [ ] Measure speed (GPS or wheel odometry)
- [ ] Verify: Speed ≤1.5 m/s (with passenger)
- [ ] Repeat test without passenger
- [ ] Verify: Speed ≤2.0 m/s (empty)

**Test Results:**
- All tests passed: ✅ Vehicle operational
- Any test failed: ❌ Vehicle out of service, investigate issue

**Tester Signature:** ________________ **Date:** __________

---

## Approval

| Role | Name | Signature | Date |
|------|------|-----------|------|
| **Project Lead** | Maeda-san | __________ | _______ |
| **Legal/Compliance Officer** | TBD | __________ | _______ |
| **Privacy Officer** | TBD | __________ | _______ |
| **Safety Engineer** | TBD | __________ | _______ |

---

**Document Metadata:**
- **Version:** 1.0
- **Created:** 2025-12-17
- **Next Review:** 2026-03-17 (quarterly review)
- **Owner:** Integration Team, Legal/Compliance Officer

---

**Related Documents:**
- `SECURITY_REQUIREMENTS.md` - 152 security requirements
- `PRIVACY_REQUIREMENTS.md` - 100 privacy requirements
- `NONFUNCTIONAL_REQUIREMENTS.md` - Compliance requirements (Section 12)
- `SECURITY_ARCHITECTURE.md` - Security controls implementation
- `ERROR_HANDLING_REQUIREMENTS.md` - Safety-critical error handling

---

*End of Document*
