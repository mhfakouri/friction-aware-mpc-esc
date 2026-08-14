# Model Notes

## Archived state-space model

The preserved `legacy_source/suspension.m` defines

\[
x =
\begin{bmatrix}
z_s & \dot z_s & z_u & \dot z_u
\end{bmatrix}^{T},
\]

with

\[
\dot x = Ax+Bu.
\]

The two inputs are interpreted from the archived matrix structure as

\[
u =
\begin{bmatrix}
F_c & z_r
\end{bmatrix}^{T},
\]

where \(F_c\) is a force acting between sprung and unsprung masses and
\(z_r\) is road displacement.

The archived simulation sets \(F_c=0\).

## Parameters

\[
m=100\ \text{kg},\qquad
m_u=10\ \text{kg},
\]

\[
k_1=k_2=200\ \text{N/m},
\qquad
c=500\ \text{N·s/m}.
\]

## State equations

The matrix model corresponds to

\[
\dot z_s = v_s,
\]

\[
m\dot v_s =
-k_1 z_s-cv_s+k_1z_u+cv_u+F_c,
\]

\[
\dot z_u=v_u,
\]

\[
m_u\dot v_u =
k_1z_s+cv_s-(k_1+k_2)z_u-cv_u-F_c+k_2z_r.
\]

## Road input

The archived indexed MATLAB construction creates two smooth bumps over a
100-s record. The cleaned runner represents the same intended shapes as
continuous half-sines:

- 0–20 s: 0.10 m peak;
- 40–60 s: 0.06 m peak.

## New analysis

The frequency response and damping sweep are modernization additions. They are
not represented as outputs from the original script.

The damping sweep intentionally illustrates a tradeoff:

- lower/intermediate damping can reduce ride acceleration;
- higher damping reduces suspension travel;
- the performance preference depends on the chosen objective.
