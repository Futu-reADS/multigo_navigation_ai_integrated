# Privacy Requirements

**Project:** Outdoor Wheelchair Transport Robot - Multi-Team System
**Document Type:** Requirements Specification
**Status:** Active
**Version:** 1.0
**Last Updated:** 2025-12-17
**Owner:** Integration Team (All Teams)

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [Data Privacy Principles](#2-data-privacy-principles)
3. [PII Collection and Processing](#3-pii-collection-and-processing)
4. [Data Subject Rights](#4-data-subject-rights)
5. [Consent Management](#5-consent-management)
6. [Data Minimization](#6-data-minimization)
7. [Purpose Limitation](#7-purpose-limitation)
8. [Data Retention and Deletion](#8-data-retention-and-deletion)
9. [Data Security and Protection](#9-data-security-and-protection)
10. [Cross-Border Data Transfers](#10-cross-border-data-transfers)
11. [Privacy by Design and Default](#11-privacy-by-design-and-default)
12. [Privacy Impact Assessment](#12-privacy-impact-assessment)
13. [Transparency and Notice](#13-transparency-and-notice)
14. [Children's Privacy](#14-childrens-privacy)
15. [Health Information Privacy (HIPAA)](#15-health-information-privacy-hipaa)
16. [Video and Biometric Data](#16-video-and-biometric-data)
17. [Third-Party Data Sharing](#17-third-party-data-sharing)
18. [Data Breach Notification](#18-data-breach-notification)
19. [Privacy Training and Awareness](#19-privacy-training-and-awareness)
20. [Compliance Auditing](#20-compliance-auditing)

---

## 1. Introduction

### 1.1 Purpose

This document specifies privacy requirements for the outdoor wheelchair transport robot fleet system. It ensures compliance with global privacy regulations:
- **GDPR** (General Data Protection Regulation - EU)
- **PIPEDA** (Personal Information Protection and Electronic Documents Act - Canada)
- **HIPAA** (Health Insurance Portability and Accountability Act - US, if applicable)
- **CCPA** (California Consumer Privacy Act - US)

### 1.2 Scope

Privacy requirements apply to all systems processing personally identifiable information (PII):
- **Team 1 (Pankaj):** Vehicle software (camera data, location tracking)
- **Team 2 (Unno):** TVM server (resident data, mission logs, operator accounts)
- **Team 3 (Maeda-san):** Hardware (data storage, physical security)

### 1.3 Definitions

**PII (Personally Identifiable Information):**
- Any information that can identify an individual, including:
  - Name, email, phone number, address
  - Medical information (diagnoses, medications, allergies)
  - Biometric data (facial recognition, if used)
  - Location data (GPS coordinates, traveled routes)
  - IP addresses, device IDs, session tokens

**Data Subject:**
- Individual whose personal data is being processed (residents, operators, caregivers)

**Data Controller:**
- Entity that determines purposes and means of processing (facility owner/operator)

**Data Processor:**
- Entity that processes data on behalf of controller (our system, potentially cloud providers)

**Sensitive Personal Data (Special Categories under GDPR Art. 9):**
- Health information
- Biometric data (for unique identification)
- Genetic data
- Racial/ethnic origin (if collected)

---

## 2. Data Privacy Principles

### REQ-PRIV-001 [CRITICAL]
**Lawfulness, Fairness, and Transparency**
- System SHALL process personal data lawfully, fairly, and transparently
- Legal basis: Consent, contract, legal obligation, or legitimate interest
- Users must be informed about data collection and use

**Rationale:** GDPR Art. 5(1)(a), PIPEDA Principle 1

### REQ-PRIV-002 [CRITICAL]
**Purpose Limitation**
- Personal data SHALL be collected for specified, explicit, and legitimate purposes
- Data SHALL NOT be processed in a manner incompatible with original purpose
- Any new purpose requires additional consent or legal basis

**Rationale:** GDPR Art. 5(1)(b)

### REQ-PRIV-003 [CRITICAL]
**Data Minimization**
- System SHALL collect only personal data adequate, relevant, and limited to what is necessary
- No excessive data collection (e.g., do not collect SSN if not needed)

**Rationale:** GDPR Art. 5(1)(c), PIPEDA Principle 4.4

### REQ-PRIV-004 [CRITICAL]
**Accuracy**
- Personal data SHALL be accurate and kept up to date
- Inaccurate data must be erased or rectified without delay
- Provide mechanisms for data subjects to update their information

**Rationale:** GDPR Art. 5(1)(d)

### REQ-PRIV-005 [CRITICAL]
**Storage Limitation**
- Personal data SHALL be kept only as long as necessary for the purposes
- Define retention periods for each data type (see Section 8)
- Automatic deletion after retention period expires

**Rationale:** GDPR Art. 5(1)(e), PIPEDA Principle 4.5

### REQ-PRIV-006 [CRITICAL]
**Integrity and Confidentiality**
- Personal data SHALL be processed securely
- Protect against unauthorized access, loss, destruction, or damage
- Implement technical and organizational measures (encryption, access control)

**Rationale:** GDPR Art. 5(1)(f), GDPR Art. 32

### REQ-PRIV-007 [CRITICAL]
**Accountability**
- Data controller SHALL be responsible for and able to demonstrate compliance
- Maintain records of processing activities
- Conduct Data Protection Impact Assessments (DPIAs) for high-risk processing

**Rationale:** GDPR Art. 5(2), GDPR Art. 24

---

## 3. PII Collection and Processing

### 3.1 Resident Data

### REQ-PRIV-008 [CRITICAL]
**Resident Profile PII**
- System SHALL collect the following PII for residents:
  - **Required:** Full name, date of birth, contact information (phone, emergency contact)
  - **Optional:** Medical conditions (allergies, mobility restrictions), photo
- Justification: Required for service delivery and safety

### REQ-PRIV-009 [CRITICAL]
**Medical Information Collection**
- Medical information (diagnoses, medications, allergies) SHALL only be collected if necessary for safe transport
- Minimize medical data: Only collect what affects transport (e.g., "requires oxygen" vs. detailed diagnosis)
- Obtain explicit consent for medical data collection (special category under GDPR Art. 9)

**Rationale:** GDPR Art. 9 (special categories require explicit consent)

### REQ-PRIV-010 [HIGH]
**Pseudonymization of Resident Data**
- Where feasible, system SHOULD pseudonymize resident data in logs and analytics
- Example: Use resident ID instead of name in telemetry logs
- Pseudonymization reduces risk (GDPR Art. 32)

**Rationale:** GDPR Art. 32(1)(a) recommends pseudonymization

### 3.2 Location and Tracking Data

### REQ-PRIV-011 [CRITICAL]
**Location Data Processing**
- System SHALL collect location data (pickup/dropoff points, traveled routes)
- Purpose: Mission execution, navigation, audit trail
- Location data is PII (can identify individuals via visited locations)

**Rationale:** GDPR Recital 26 (location data can identify individuals)

### REQ-PRIV-012 [HIGH]
**Location Data Retention**
- Precise location data (GPS coordinates) SHALL be retained for maximum 90 days
- After 90 days, aggregate to zone level (e.g., "Building A" instead of exact coordinates)
- Exception: Retain longer if required by law (e.g., accident investigation)

**Rationale:** Data minimization (GDPR Art. 5(1)(c))

### REQ-PRIV-013 [HIGH]
**Opt-Out for Location Tracking**
- System SHOULD allow residents to opt out of detailed location tracking (if legally permissible)
- Minimum viable tracking: Pickup/dropoff points only (no route details)
- Note: Some facilities may require tracking for liability (inform resident)

**Rationale:** CCPA right to opt-out, PIPEDA consent principle

### 3.3 Camera and Video Data

### REQ-PRIV-014 [CRITICAL]
**Camera Data Purpose**
- Vehicle cameras SHALL be used solely for:
  1. Obstacle detection (navigation safety)
  2. Incident investigation (accidents, vandalism)
- Camera data SHALL NOT be used for facial recognition or marketing

**Rationale:** Purpose limitation (GDPR Art. 5(1)(b))

### REQ-PRIV-015 [CRITICAL]
**Camera Data Retention**
- Camera footage SHALL be deleted after 7 days (unless incident flagged)
- If incident: Retain footage for 90 days or until investigation complete
- No continuous recording of passengers (only exterior cameras active during transport)

**Rationale:** Storage limitation (GDPR Art. 5(1)(e))

### REQ-PRIV-016 [HIGH]
**Camera Signage**
- Vehicle SHALL display visible signage: "This vehicle is equipped with cameras for safety purposes"
- Signage must be readable from 3 meters
- Include contact information for privacy inquiries

**Rationale:** Transparency (GDPR Art. 5(1)(a)), PIPEDA Principle 4.8

### REQ-PRIV-017 [HIGH]
**Camera Data Encryption**
- Camera footage stored on vehicle SHALL be encrypted (AES-256)
- Footage transmitted to TVM SHALL use TLS 1.3
- Decryption key accessible only to authorized personnel (security team, incident investigators)

**Rationale:** Security of processing (GDPR Art. 32)

### 3.4 Operator and Caregiver Data

### REQ-PRIV-018 [HIGH]
**Operator Account Data**
- System SHALL collect operator PII:
  - **Required:** Full name, email, role (for access control)
  - **Optional:** Phone number (for notifications)
- No excessive data (e.g., home address not required)

**Rationale:** Data minimization (GDPR Art. 5(1)(c))

### REQ-PRIV-019 [HIGH]
**Operator Activity Logging**
- System SHALL log operator actions (mission creation, vehicle control, data access)
- Purpose: Audit trail, accountability, incident investigation
- Logs retained for 1 year (compliance with audit requirements)

**Rationale:** Accountability (GDPR Art. 5(2)), HIPAA audit controls

### REQ-PRIV-020 [MEDIUM]
**Operator Privacy from Employer**
- System SHOULD NOT expose detailed operator activity to employer without justification
- Example: Log "Operator A created 15 missions" but not "Operator A took 30-minute break at 2 PM"
- Balance: Accountability vs. employee privacy

**Rationale:** PIPEDA Principle 4.4 (limiting collection), employee privacy rights

---

## 4. Data Subject Rights

### 4.1 Right to Access (GDPR Art. 15, PIPEDA Principle 4.9)

### REQ-PRIV-021 [CRITICAL]
**Right to Access Personal Data**
- System SHALL provide data subjects with access to their personal data upon request
- Within 30 days of request (GDPR: 1 month, extendable to 3 months)
- Provide data in structured, commonly used format (JSON or PDF)

**Implementation:** API endpoint `/api/gdpr/access/:user_id`

### REQ-PRIV-022 [HIGH]
**Access Request Authentication**
- System SHALL verify identity of data subject before providing access
- Authentication: Login + MFA, or verified email + identity document
- Prevent unauthorized access to others' data

**Rationale:** Security measure (GDPR Art. 32)

### 4.2 Right to Rectification (GDPR Art. 16)

### REQ-PRIV-023 [CRITICAL]
**Right to Correct Inaccurate Data**
- System SHALL allow data subjects to correct inaccurate personal data
- Within 30 days of request
- Notify third parties if data was shared (e.g., backup systems)

**Implementation:** User profile edit page, or manual request to admin

### REQ-PRIV-024 [HIGH]
**Audit Trail for Corrections**
- System SHALL log all data corrections (who changed what, when)
- Purpose: Accountability, fraud prevention
- Log retention: Same as original data retention period

**Rationale:** Accountability (GDPR Art. 5(2))

### 4.3 Right to Erasure (GDPR Art. 17, CCPA Right to Delete)

### REQ-PRIV-025 [CRITICAL]
**Right to Be Forgotten**
- System SHALL delete personal data upon request (with exceptions)
- Within 30 days of request
- Exceptions (SHALL NOT delete if):
  1. Legal obligation to retain (e.g., tax records, accident reports)
  2. Establishment, exercise, or defense of legal claims
  3. Public interest (e.g., safety investigation)

**Implementation:** API endpoint `/api/gdpr/delete/:user_id`

### REQ-PRIV-026 [CRITICAL]
**Anonymization (Not Deletion)**
- If deletion not possible (legal hold), system SHALL anonymize data
- Remove all identifiers: Name → "Deleted User #12345", email → null
- Retain non-PII data for analytics (e.g., "Mission count: 50" without user identity)

**Rationale:** GDPR Art. 17(3) exceptions, data minimization

### REQ-PRIV-027 [HIGH]
**Cascading Deletion**
- Deletion SHALL cascade to all related data:
  - Resident profile, mission history, telemetry logs, camera footage (if resident visible)
- Verify deletion across backups (restore backup, re-delete if needed)

**Rationale:** Complete erasure (GDPR Art. 17)

### REQ-PRIV-028 [HIGH]
**Deletion Audit Log**
- System SHALL log all deletion requests (who requested, when, what was deleted)
- Audit log retained even after data deleted (for compliance verification)
- Log retention: 7 years (matches regulatory compliance requirements)

**Rationale:** Accountability (GDPR Art. 5(2))

### 4.4 Right to Data Portability (GDPR Art. 20)

### REQ-PRIV-029 [HIGH]
**Export Data in Portable Format**
- System SHALL export data subject's personal data in machine-readable format
- Format: JSON (structured, commonly used)
- Include: Resident profile, mission history, preferences
- Within 30 days of request

**Implementation:** API endpoint `/api/gdpr/export/:user_id`

### REQ-PRIV-030 [MEDIUM]
**Direct Transfer to Another Controller**
- System SHOULD support direct transfer of data to another system (if technically feasible)
- Example: Send JSON export to another fleet management system via API
- Only if both systems support compatible formats

**Rationale:** GDPR Art. 20(2) (right to transmit directly)

### 4.5 Right to Object (GDPR Art. 21)

### REQ-PRIV-031 [HIGH]
**Right to Object to Processing**
- Data subject SHALL be able to object to processing based on legitimate interest
- Example: "I don't want my data used for system analytics"
- System must stop processing unless compelling legitimate grounds

**Implementation:** "Opt out of analytics" checkbox in user profile

### REQ-PRIV-032 [HIGH]
**Right to Object to Direct Marketing**
- System SHALL provide opt-out for marketing communications (if any)
- Opt-out mechanism: Unsubscribe link in emails, or account settings toggle
- Honored immediately (no delay)

**Rationale:** GDPR Art. 21(3) (absolute right to object to marketing)

### 4.6 Right to Restrict Processing (GDPR Art. 18)

### REQ-PRIV-033 [MEDIUM]
**Temporary Suspension of Processing**
- Data subject MAY request restriction of processing in certain cases:
  1. Contesting accuracy of data (until verified)
  2. Processing unlawful, but subject opposes deletion
  3. Legal claim defense
- System SHALL mark data as "restricted" (do not process, but do not delete)

**Implementation:** Database flag `processing_restricted = true`

---

## 5. Consent Management

### REQ-PRIV-034 [CRITICAL]
**Explicit Consent for Medical Data**
- System SHALL obtain explicit consent for processing medical information (special category)
- Consent must be:
  - **Freely given:** No coercion (service not conditional on consent if not necessary)
  - **Specific:** Clear purpose stated ("for safe transport")
  - **Informed:** Privacy notice provided
  - **Unambiguous:** Affirmative action (checkbox, not pre-checked)

**Rationale:** GDPR Art. 9(2)(a) (explicit consent for special categories)

### REQ-PRIV-035 [CRITICAL]
**Consent for Video Recording**
- If vehicle has interior cameras (recording passengers), obtain explicit consent
- Consent banner: "By using this vehicle, you consent to video recording for safety purposes"
- Provide opt-out (alternative transport without cameras, if available)

**Rationale:** Privacy expectation, biometric data if facial recognition used

### REQ-PRIV-036 [HIGH]
**Consent for Data Sharing**
- System SHALL obtain consent before sharing personal data with third parties (e.g., analytics vendors)
- Exception: No consent needed for service providers (data processors) if DPA in place
- Granular consent: Separate checkboxes for different purposes

**Rationale:** GDPR Art. 6(1)(a) (consent as legal basis)

### REQ-PRIV-037 [HIGH]
**Withdrawal of Consent**
- Data subject SHALL be able to withdraw consent at any time
- Withdrawal must be as easy as giving consent (e.g., same checkbox, or account settings)
- System stops processing within 7 days of withdrawal

**Rationale:** GDPR Art. 7(3) (right to withdraw consent)

### REQ-PRIV-038 [HIGH]
**Consent Records**
- System SHALL maintain records of all consents:
  - Who gave consent, when, for what purpose, version of privacy notice shown
- Consent log retained for 7 years (compliance audit)

**Rationale:** Accountability (GDPR Art. 5(2))

### REQ-PRIV-039 [MEDIUM]
**Consent Refresh**
- System SHOULD re-obtain consent every 24 months
- Purpose: Ensure ongoing awareness of data processing
- Re-consent prompt in dashboard: "Your privacy preferences were last updated 2 years ago. Please review."

**Rationale:** Best practice (not legally required, but recommended)

---

## 6. Data Minimization

### REQ-PRIV-040 [CRITICAL]
**Collect Only Necessary Data**
- System SHALL collect only data necessary for the specified purpose
- Do NOT collect:
  - Social Security Number (unless legally required)
  - Financial information (unless payment processing needed)
  - Unnecessary medical details (e.g., full medical history if only mobility restriction relevant)

**Rationale:** GDPR Art. 5(1)(c), PIPEDA Principle 4.4

### REQ-PRIV-041 [HIGH]
**Default Privacy Settings**
- System SHALL configure privacy settings to most protective by default
- Example: Detailed location tracking OFF by default (enable only if user opts in)
- Example: Marketing emails OFF by default

**Rationale:** Privacy by default (GDPR Art. 25(2))

### REQ-PRIV-042 [HIGH]
**Optional Fields**
- System SHALL clearly mark optional fields in data collection forms
- Example: "Email address (optional)" vs. "Full name (required)"
- Do not make fields required if not necessary for service

**Rationale:** Data minimization (GDPR Art. 5(1)(c))

### REQ-PRIV-043 [MEDIUM]
**Data Collection Review**
- Organization SHALL review data collection practices annually
- Question: "Is this data still necessary?"
- Remove unnecessary fields from forms

**Rationale:** Accountability (GDPR Art. 5(2))

---

## 7. Purpose Limitation

### REQ-PRIV-044 [CRITICAL]
**Specified Purpose for Data Collection**
- System SHALL define and document purpose for each type of personal data collected
- Example purposes:
  - Resident name: Service delivery (mission creation)
  - Location data: Navigation, audit trail
  - Camera footage: Safety, incident investigation

**Rationale:** GDPR Art. 5(1)(b)

### REQ-PRIV-045 [CRITICAL]
**No Secondary Use Without Consent**
- Personal data SHALL NOT be used for purposes incompatible with original purpose
- Example: Cannot use resident medical data for marketing without new consent
- Exception: Compatible purpose (e.g., improving safety features is compatible with original safety purpose)

**Rationale:** GDPR Art. 5(1)(b)

### REQ-PRIV-046 [HIGH]
**Purpose Documentation**
- System SHALL maintain a "Record of Processing Activities" (ROPA)
- Document for each data type:
  - What is collected
  - Why (purpose)
  - Legal basis (consent, contract, legitimate interest)
  - Retention period
  - Who has access

**Rationale:** GDPR Art. 30 (records of processing activities)

**Implementation:** Create `ROPA.md` document

---

## 8. Data Retention and Deletion

### 8.1 Retention Periods

### REQ-PRIV-047 [CRITICAL]
**Retention Periods by Data Type**
- System SHALL enforce the following retention periods:

| Data Type | Retention Period | Justification |
|-----------|------------------|---------------|
| **Resident Profile** | Active + 3 years after last service | Contractual obligation, re-enrollment |
| **Mission Logs** | 3 years | Audit trail, liability |
| **Telemetry Data** | 90 days | Operational diagnostics |
| **Camera Footage** | 7 days (30 days if incident) | Safety, incident investigation |
| **Location Data (precise)** | 90 days | Navigation audit, then aggregate |
| **Audit Logs** | 7 years | Compliance (ISO 27001, HIPAA) |
| **Medical Information** | 7 years (if HIPAA applies) | HIPAA requirement (6 years + 1 buffer) |
| **Consent Records** | 7 years | Compliance verification |
| **Financial Records** | 7 years | Tax law (varies by jurisdiction) |

**Rationale:** Storage limitation (GDPR Art. 5(1)(e)), legal requirements

### REQ-PRIV-048 [CRITICAL]
**Automatic Deletion**
- System SHALL automatically delete data after retention period expires
- Scheduled job runs daily at 2 AM UTC: Delete expired data
- Deletion SHALL be irreversible (no "soft delete" where data still accessible)

**Rationale:** Storage limitation (GDPR Art. 5(1)(e))

### REQ-PRIV-049 [HIGH]
**Deletion from Backups**
- Deleted data SHALL be removed from backups within 90 days
- Implementation: Restore backup, re-apply deletions, or wait for backup to age out
- Exception: Archival backups (7-year retention for compliance) may retain anonymized data

**Rationale:** Complete erasure (GDPR Art. 17)

### REQ-PRIV-050 [MEDIUM]
**Deletion Verification**
- System SHOULD provide deletion verification report
- Report: "Data for Resident #123 deleted on 2025-12-17, verified across 3 database replicas and 2 backup generations"
- Available to data subject upon request

**Rationale:** Accountability (GDPR Art. 5(2))

### 8.2 Legal Hold

### REQ-PRIV-051 [CRITICAL]
**Legal Hold Override**
- System SHALL support "legal hold" to prevent deletion when required by law
- Example: Pending lawsuit requires preserving mission logs beyond normal retention
- Legal hold flag: `legal_hold = true` (prevents automatic deletion)

**Rationale:** GDPR Art. 17(3)(e) (legal claims exception)

### REQ-PRIV-052 [HIGH]
**Legal Hold Authorization**
- Legal hold SHALL only be applied by authorized personnel (Legal Counsel, Compliance Officer)
- Audit log: Who applied legal hold, when, reason
- Legal hold reviewed quarterly (is it still necessary?)

**Rationale:** Accountability, prevent abuse

---

## 9. Data Security and Protection

*(Note: Detailed security requirements are in SECURITY_REQUIREMENTS.md. This section highlights privacy-specific security requirements.)*

### REQ-PRIV-053 [CRITICAL]
**Encryption of PII at Rest**
- All PII SHALL be encrypted at rest using AES-256 or stronger
- Includes: Resident profiles, medical info, mission logs, operator accounts
- See SECURITY_REQUIREMENTS.md REQ-SEC-032

**Rationale:** GDPR Art. 32(1)(a) (encryption recommended)

### REQ-PRIV-054 [CRITICAL]
**Encryption in Transit**
- All PII transmitted over network SHALL use TLS 1.3 (or TLS 1.2 minimum)
- No unencrypted transmission of PII (e.g., no HTTP, only HTTPS)
- See SECURITY_REQUIREMENTS.md REQ-SEC-038

**Rationale:** GDPR Art. 32 (security of processing)

### REQ-PRIV-055 [CRITICAL]
**Access Control for PII**
- Access to PII SHALL be restricted based on role (RBAC)
- Principle of least privilege: Users access only data needed for their job
- Example: Nurse can view residents they care for, but not all residents
- See SECURITY_REQUIREMENTS.md REQ-SEC-017

**Rationale:** GDPR Art. 32(1)(b) (confidentiality)

### REQ-PRIV-056 [HIGH]
**Audit Logging of PII Access**
- All access to PII SHALL be logged (who accessed what, when)
- Purpose: Detect unauthorized access, accountability
- Log retention: 1 year minimum
- See SECURITY_REQUIREMENTS.md REQ-SEC-092

**Rationale:** GDPR Art. 32(1)(d) (ability to ensure ongoing confidentiality)

### REQ-PRIV-057 [HIGH]
**Pseudonymization**
- System SHOULD pseudonymize PII where feasible
- Example: Telemetry logs use resident ID instead of name
- Reduces risk: Even if logs leaked, no direct identification
- See SECURITY_REQUIREMENTS.md (related to data protection)

**Rationale:** GDPR Art. 32(1)(a) recommends pseudonymization

### REQ-PRIV-058 [HIGH]
**Data Loss Prevention (DLP)**
- System SHOULD implement DLP controls to prevent accidental PII disclosure
- Example: Block email attachments containing large amounts of PII
- Example: Redact PII in logs sent to external support (e.g., error reports to vendors)

**Rationale:** Security of processing (GDPR Art. 32)

---

## 10. Cross-Border Data Transfers

### REQ-PRIV-059 [CRITICAL]
**Identify Data Transfer Locations**
- System SHALL document where personal data is stored and processed
- Example: Data at rest in Canada (AWS ca-central-1), backups in US (AWS us-east-1)
- If data leaves EEA (for EU residents), special rules apply (GDPR Chapter V)

**Rationale:** GDPR Art. 44 (transfers to third countries)

### REQ-PRIV-060 [CRITICAL]
**Adequacy Decision for Transfers**
- If transferring EU personal data to third country, ensure adequate protection:
  - **Option 1:** Transfer to country with adequacy decision (EU-approved, e.g., Canada, Japan)
  - **Option 2:** Use Standard Contractual Clauses (SCCs)
  - **Option 3:** Binding Corporate Rules (BCRs)

**Rationale:** GDPR Art. 45 (adequacy decision), GDPR Art. 46 (appropriate safeguards)

### REQ-PRIV-061 [HIGH]
**Data Residency (If Required)**
- If jurisdiction requires data residency (e.g., Canada PIPEDA, Russia Federal Law 242), store data in-country
- Example: For Canadian residents, store data in AWS ca-central-1 (Canada region)
- No replication to non-Canadian regions without consent

**Rationale:** PIPEDA, local data protection laws

### REQ-PRIV-062 [HIGH]
**Cloud Provider Data Processing Agreement (DPA)**
- If using cloud provider (AWS, Azure, GCP), sign Data Processing Agreement
- DPA must comply with GDPR Art. 28 (processor obligations)
- Verify: Provider only processes data per controller's instructions

**Rationale:** GDPR Art. 28 (processor requirements)

### REQ-PRIV-063 [MEDIUM]
**Subprocessor Notification**
- If cloud provider uses subprocessors (e.g., AWS subcontracts data center operations), maintain list
- Data subjects have right to object to subprocessors
- Notification: Update privacy policy with subprocessor list

**Rationale:** GDPR Art. 28(2) (subprocessor authorization)

---

## 11. Privacy by Design and Default

### REQ-PRIV-064 [CRITICAL]
**Privacy by Design**
- Privacy SHALL be considered in all system design decisions
- Privacy Impact Assessment (PIA) required for new features involving PII
- Integrate privacy from project inception (not retrofit)

**Rationale:** GDPR Art. 25(1) (data protection by design)

### REQ-PRIV-065 [CRITICAL]
**Privacy by Default**
- System SHALL use most privacy-protective settings by default
- Example: Location tracking precision set to "zone level" by default (opt-in for precise coordinates)
- Example: Marketing emails OFF by default (opt-in to enable)

**Rationale:** GDPR Art. 25(2) (data protection by default)

### REQ-PRIV-066 [HIGH]
**Privacy Features**
- System SHOULD include privacy-enhancing features:
  - Data export (portability)
  - Data deletion (right to erasure)
  - Consent management dashboard
  - Privacy settings page

**Rationale:** User empowerment, transparency

### REQ-PRIV-067 [HIGH]
**Privacy Training for Developers**
- All developers SHALL complete privacy training annually
- Topics: GDPR basics, data minimization, secure coding for PII
- Certification: Completion tracked by HR

**Rationale:** Accountability (GDPR Art. 5(2))

---

## 12. Privacy Impact Assessment

### REQ-PRIV-068 [CRITICAL]
**DPIA for High-Risk Processing**
- Data Protection Impact Assessment (DPIA) required for:
  1. Systematic monitoring (location tracking, video surveillance)
  2. Large-scale processing of special categories (medical data)
  3. Automated decision-making with legal effects
- DPIA completed before processing begins

**Rationale:** GDPR Art. 35 (DPIA for high-risk processing)

### REQ-PRIV-069 [CRITICAL]
**DPIA Content**
- DPIA SHALL include:
  1. Description of processing operations and purposes
  2. Assessment of necessity and proportionality
  3. Assessment of risks to data subjects' rights
  4. Mitigation measures (security controls, privacy features)
  5. Approval by Data Protection Officer (DPO) or legal counsel

**Rationale:** GDPR Art. 35(7)

### REQ-PRIV-070 [HIGH]
**DPIA Review Frequency**
- DPIA SHALL be reviewed:
  - Annually (scheduled review)
  - When processing operations change significantly
  - After privacy incident
- Updated DPIA version controlled (track changes over time)

**Rationale:** Ongoing accountability (GDPR Art. 35(11))

### REQ-PRIV-071 [MEDIUM]
**Consult Supervisory Authority (If High Risk)**
- If DPIA identifies high residual risk (after mitigation), consult data protection authority
- Example: EU supervisory authority (e.g., CNIL in France, ICO in UK)
- Required before processing begins

**Rationale:** GDPR Art. 36 (prior consultation)

---

## 13. Transparency and Notice

### REQ-PRIV-072 [CRITICAL]
**Privacy Notice (Privacy Policy)**
- System SHALL provide clear, accessible Privacy Notice to data subjects
- Content:
  1. Identity of data controller (organization name, contact)
  2. Data Protection Officer contact (if appointed)
  3. Purposes of processing and legal basis
  4. Categories of personal data collected
  5. Recipients of data (third parties, if any)
  6. Data retention periods
  7. Data subject rights (access, rectification, erasure, etc.)
  8. Right to lodge complaint with supervisory authority
  9. Whether data used for automated decision-making

**Rationale:** GDPR Art. 13 (information to be provided), PIPEDA Principle 4.8

### REQ-PRIV-073 [CRITICAL]
**Privacy Notice Accessibility**
- Privacy Notice SHALL be:
  - **Prominent:** Link on homepage, registration page, mobile app
  - **Clear language:** Avoid legal jargon (8th-grade reading level)
  - **Layered approach:** Summary (1 page) + full notice
  - **Multiple languages:** English + local language (French for Canada, etc.)

**Rationale:** GDPR Art. 12(1) (transparent, intelligible language)

### REQ-PRIV-074 [HIGH]
**Just-in-Time Notices**
- System SHOULD provide contextual privacy notices at point of data collection
- Example: When enabling location tracking, show tooltip: "Your location is used for navigation and stored for 90 days"
- More effective than relying solely on privacy policy

**Rationale:** Transparency (GDPR Art. 5(1)(a))

### REQ-PRIV-075 [HIGH]
**Privacy Notice Version Control**
- System SHALL maintain version history of Privacy Notice
- When updated, notify users: "Our privacy policy has been updated. Please review changes."
- User must acknowledge new policy (click "I accept") for material changes

**Rationale:** GDPR Art. 13(3) (inform of changes)

### REQ-PRIV-076 [MEDIUM]
**Cookie/Tracking Notice (If Applicable)**
- If website uses cookies or trackers, provide cookie notice
- Comply with ePrivacy Directive (EU Cookie Law): Obtain consent before non-essential cookies
- Cookie banner: "This site uses cookies for analytics. [Accept] [Reject] [Customize]"

**Rationale:** ePrivacy Directive Art. 5(3), GDPR Art. 7

---

## 14. Children's Privacy

### REQ-PRIV-077 [CRITICAL]
**Age Verification**
- If system collects data from children (<16 in EU, <13 in US under COPPA), implement age verification
- Example: Date of birth field, block registration if under age
- If child: Require parental consent

**Rationale:** GDPR Art. 8 (child's consent), COPPA (US)

### REQ-PRIV-078 [HIGH]
**Parental Consent Mechanism**
- If children use system, obtain verifiable parental consent
- Methods: Email verification from parent, credit card verification, video call
- High bar: COPPA requires "verifiable" (not just checkbox)

**Rationale:** COPPA § 312.5 (parental consent)

### REQ-PRIV-079 [MEDIUM]
**Enhanced Privacy for Children**
- If children use system, apply enhanced privacy protections:
  - No behavioral advertising
  - Minimize data collection (only essential)
  - No data sharing with third parties (except service providers)

**Rationale:** GDPR Recital 38 (special protection for children)

**Note:** Elderly care facilities unlikely to serve children, but include if system expands to pediatric facilities.

---

## 15. Health Information Privacy (HIPAA)

*(Applicable if system operates in US and processes Protected Health Information (PHI))*

### REQ-PRIV-080 [CRITICAL]
**HIPAA Compliance (If Applicable)**
- If system is "Business Associate" (processes PHI on behalf of Covered Entity), comply with HIPAA
- Sign Business Associate Agreement (BAA) with Covered Entity (healthcare provider)
- Implement HIPAA Security Rule: Administrative, Physical, Technical safeguards

**Rationale:** HIPAA Privacy Rule (45 CFR § 164.502)

### REQ-PRIV-081 [CRITICAL]
**Minimum Necessary Standard**
- Access to PHI SHALL be limited to minimum necessary for job function
- Example: Driver does not need access to resident's medical diagnosis (only mobility restrictions)
- Document: Access control policies, role definitions

**Rationale:** HIPAA Minimum Necessary Standard (45 CFR § 164.502(b))

### REQ-PRIV-082 [HIGH]
**HIPAA Breach Notification**
- If PHI breach occurs, notify:
  1. Affected individuals (within 60 days)
  2. HHS Secretary (if >500 individuals affected, immediate; if <500, annual report)
  3. Media (if >500 individuals in same state/jurisdiction)
- Breach definition: Unauthorized acquisition, access, use, or disclosure of PHI

**Rationale:** HIPAA Breach Notification Rule (45 CFR § 164.404-414)

### REQ-PRIV-083 [MEDIUM]
**Patient Rights under HIPAA**
- If HIPAA applies, provide patient rights:
  - Right to access PHI (within 30 days)
  - Right to amend PHI
  - Right to accounting of disclosures (who accessed PHI)
  - Right to request restrictions on use/disclosure

**Rationale:** HIPAA Privacy Rule (45 CFR § 164.524-528)

---

## 16. Video and Biometric Data

### REQ-PRIV-084 [CRITICAL]
**Biometric Data Consent (If Facial Recognition Used)**
- If system uses facial recognition (biometric data), obtain explicit consent
- Biometric data is "special category" under GDPR Art. 9 (requires explicit consent)
- Consent form: "I consent to facial recognition for [purpose]"

**Rationale:** GDPR Art. 9(2)(a) (explicit consent for biometric data)

### REQ-PRIV-085 [HIGH]
**Facial Recognition Purpose Limitation**
- Facial recognition SHALL only be used for specified purposes:
  - Example: Resident identification at pickup (verify correct person)
  - NOT for: Marketing, profiling, law enforcement (without warrant)

**Rationale:** Purpose limitation (GDPR Art. 5(1)(b))

### REQ-PRIV-086 [HIGH]
**Biometric Data Retention**
- Facial recognition templates (biometric data) SHALL be deleted after purpose fulfilled
- Example: Resident face template deleted immediately after identity verified
- Exception: Resident profile photo (retained with consent) is separate from biometric template

**Rationale:** Storage limitation (GDPR Art. 5(1)(e))

### REQ-PRIV-087 [MEDIUM]
**Video Redaction (Privacy Protection)**
- System SHOULD support video redaction (blur faces) for non-relevant individuals
- Example: Bystanders in camera footage blurred before footage shared with investigators
- Reduces privacy impact

**Rationale:** Data minimization (GDPR Art. 5(1)(c))

---

## 17. Third-Party Data Sharing

### REQ-PRIV-088 [CRITICAL]
**No Sale of Personal Data**
- System SHALL NOT sell personal data to third parties
- Exception: If user explicitly consents (e.g., opt-in to data marketplace)

**Rationale:** CCPA § 1798.115 (right to opt-out of sale), PIPEDA Principle 4.5

### REQ-PRIV-089 [CRITICAL]
**Data Processing Agreement with Vendors**
- Any third-party vendor processing personal data (service providers, cloud providers) SHALL sign Data Processing Agreement (DPA)
- DPA must include:
  - Scope of processing
  - Security obligations
  - Subprocessor authorization
  - Audit rights

**Rationale:** GDPR Art. 28 (processor requirements)

### REQ-PRIV-090 [HIGH]
**Vendor Privacy Due Diligence**
- Before engaging vendor, conduct privacy due diligence:
  - Review vendor's privacy policy and security practices
  - Verify compliance with GDPR/PIPEDA (if handling EU/Canadian data)
  - Verify data residency (data stored in approved locations)
- Document: Vendor assessment checklist

**Rationale:** Accountability (GDPR Art. 5(2))

### REQ-PRIV-091 [HIGH]
**Limit Data Shared with Vendors**
- Share only data necessary for vendor to perform service
- Example: If vendor provides mapping service, share location data but NOT medical information
- Anonymize or pseudonymize data if possible

**Rationale:** Data minimization (GDPR Art. 5(1)(c))

---

## 18. Data Breach Notification

### REQ-PRIV-092 [CRITICAL]
**Breach Notification to Supervisory Authority**
- If personal data breach occurs, notify data protection authority within 72 hours
- Example: EU supervisory authority (e.g., CNIL, ICO), Canadian OPC (Office of the Privacy Commissioner)
- Breach definition: Unauthorized access, loss, destruction, or disclosure of personal data

**Rationale:** GDPR Art. 33 (notification to supervisory authority)

### REQ-PRIV-093 [CRITICAL]
**Breach Notification to Data Subjects**
- If breach poses "high risk" to data subjects' rights, notify affected individuals without undue delay
- High risk examples: Medical data leaked, financial data compromised, identity theft risk
- Notification content: Nature of breach, likely consequences, mitigation measures, contact point

**Rationale:** GDPR Art. 34 (notification to data subject)

### REQ-PRIV-094 [HIGH]
**Breach Documentation**
- System SHALL maintain record of all data breaches (even if not notifiable)
- Document: Date, nature of breach, affected data, consequences, remedial actions
- Record retained for 7 years (compliance audit)

**Rationale:** GDPR Art. 33(5) (documentation of breaches)

### REQ-PRIV-095 [HIGH]
**Breach Response Plan**
- Organization SHALL have documented Data Breach Response Plan
- Plan includes: Roles, notification procedures, communication templates
- Plan tested annually (tabletop exercise)

**Rationale:** Accountability (GDPR Art. 5(2))

---

## 19. Privacy Training and Awareness

### REQ-PRIV-096 [HIGH]
**Privacy Training for All Staff**
- All staff with access to personal data SHALL complete privacy training annually
- Topics: GDPR/PIPEDA basics, data handling, incident reporting
- Completion tracked and audited

**Rationale:** Accountability (GDPR Art. 5(2))

### REQ-PRIV-097 [MEDIUM]
**Role-Specific Training**
- Staff with high-risk roles SHALL complete advanced training:
  - Developers: Secure coding for PII, privacy by design
  - Operators: Data access controls, confidentiality
  - Admins: Data breach response, incident management

**Rationale:** Risk-based approach

### REQ-PRIV-098 [MEDIUM]
**Privacy Awareness Campaign**
- Organization SHOULD run annual privacy awareness campaign
- Examples: Privacy posters, email reminders, privacy champion program
- Goal: Foster privacy culture

**Rationale:** Best practice

---

## 20. Compliance Auditing

### REQ-PRIV-099 [CRITICAL]
**Annual Privacy Audit**
- Organization SHALL conduct annual privacy compliance audit
- Scope: Review data processing activities, consent records, access logs, retention policies
- Auditor: Internal compliance team or external auditor
- Findings documented and remediated within 90 days

**Rationale:** Accountability (GDPR Art. 5(2))

### REQ-PRIV-100 [HIGH]
**Data Protection Officer (DPO) - If Required**
- If GDPR applies and organization meets criteria (public authority, large-scale monitoring, or large-scale special categories), appoint Data Protection Officer
- DPO responsibilities: Monitor compliance, advise on DPIAs, cooperate with supervisory authority
- DPO contact info published in Privacy Notice

**Rationale:** GDPR Art. 37 (designation of DPO)

---

## Summary

**Total Requirements:** 100 (CRITICAL: 42, HIGH: 46, MEDIUM: 12)

### Requirements by Category

| Category | Count | Critical | High | Medium |
|----------|-------|----------|------|--------|
| **Data Privacy Principles** | 7 | 7 | 0 | 0 |
| **PII Collection and Processing** | 13 | 6 | 5 | 2 |
| **Data Subject Rights** | 13 | 7 | 5 | 1 |
| **Consent Management** | 6 | 2 | 3 | 1 |
| **Data Minimization** | 4 | 1 | 2 | 1 |
| **Purpose Limitation** | 3 | 2 | 1 | 0 |
| **Data Retention and Deletion** | 6 | 3 | 2 | 1 |
| **Data Security and Protection** | 6 | 3 | 3 | 0 |
| **Cross-Border Data Transfers** | 5 | 2 | 2 | 1 |
| **Privacy by Design and Default** | 4 | 2 | 2 | 0 |
| **Privacy Impact Assessment** | 4 | 2 | 1 | 1 |
| **Transparency and Notice** | 5 | 2 | 2 | 1 |
| **Children's Privacy** | 3 | 1 | 1 | 1 |
| **Health Information Privacy** | 4 | 2 | 1 | 1 |
| **Video and Biometric Data** | 4 | 1 | 2 | 1 |
| **Third-Party Data Sharing** | 4 | 2 | 2 | 0 |
| **Data Breach Notification** | 4 | 2 | 2 | 0 |
| **Privacy Training and Awareness** | 3 | 0 | 1 | 2 |
| **Compliance Auditing** | 2 | 1 | 1 | 0 |

---

## Compliance Checklist

**Before Production Deployment:**
- [ ] Privacy Notice published and accessible
- [ ] Consent mechanisms implemented (medical data, video recording)
- [ ] Data subject rights portal (access, rectification, erasure, portability)
- [ ] Retention policies configured (automatic deletion after retention periods)
- [ ] Encryption at rest and in transit enabled
- [ ] RBAC access controls implemented
- [ ] Audit logging enabled for all PII access
- [ ] DPIA completed for high-risk processing (location tracking, medical data)
- [ ] DPA signed with cloud provider and other vendors
- [ ] Data breach response plan documented and tested
- [ ] Privacy training completed by all staff

---

## Related Documents

- `SECURITY_REQUIREMENTS.md` - Technical security requirements (encryption, access control, audit logging)
- `SECURITY_ARCHITECTURE.md` - Security architecture implementing privacy-supporting controls
- `REGULATORY_COMPLIANCE_REQUIREMENTS.md` (to be created) - Detailed compliance mapping to GDPR, PIPEDA, HIPAA, CCPA
- `ROPA.md` (to be created) - Record of Processing Activities (GDPR Art. 30 requirement)

---

**Document Metadata:**
- **Version:** 1.0
- **Created:** 2025-12-17
- **Next Review:** 2026-03-17 (quarterly review, or when regulations change)
- **Owner:** Integration Team (All Teams), Legal/Compliance Officer

---

**Approval Signatures:**

| Role | Name | Signature | Date |
|------|------|-----------|------|
| **Project Lead** | Maeda-san | __________ | _______ |
| **Legal/Compliance** | TBD | __________ | _______ |
| **Data Protection Officer** | TBD (if applicable) | __________ | _______ |

---

*End of Document*
