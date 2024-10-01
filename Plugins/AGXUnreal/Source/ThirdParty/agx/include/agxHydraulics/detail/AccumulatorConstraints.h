/*
Copyright 2007-2024. Algoryx Simulation AB.

All AGX source code, intellectual property, documentation, sample code,
tutorials, scene files and technical white papers, are copyrighted, proprietary
and confidential material of Algoryx Simulation AB. You may not download, read,
store, distribute, publish, copy or otherwise disseminate, use or expose this
material unless having a written signed agreement with Algoryx Simulation AB, or having been
advised so by Algoryx Simulation AB for a time limited evaluation, or having purchased a
valid commercial license from Algoryx Simulation AB.

Algoryx Simulation AB disclaims all responsibilities for loss or damage caused
from using this software, unless otherwise stated in written agreements with
Algoryx Simulation AB.
*/


#ifndef AGXHYDRAULICS_DETAIL_ACCUMULATOR_CONSTRAINTS_H
#define AGXHYDRAULICS_DETAIL_ACCUMULATOR_CONSTRAINTS_H


#include <agxHydraulics/export.h>

#include <agx/Constraint.h>
#include <agx/ConstraintImplementation.h>



namespace agxHydraulics
{
  class Accumulator;

  /// \cond INTERNAL_DOCUMENTATION
  namespace detail
  {

    /*
    A set of non-PowerLine constraints used by the Accumulator. Acts to keep
    the volume inside a given bound and provides a resisting pressure that
    increases with the contained volume.
    */


    /**
    Communication interface between the constraint implementation and the
    various elementary constraints. Allows access to the actual data, which is
    located in a central data store.
    */
    class AGXHYDRAULICS_EXPORT ElementaryAccumulatorConstraintData : public agx::ElementaryConstraintData
    {
      public:
        ElementaryAccumulatorConstraintData();

        /**
        \param length - The length/height of the accumulator.
        \param area - The area of the accumulator. Full volume of accumulator is length*area.
        \param fluidVolume - The amount of fluid currently in the accumulator.
        \param accumulatorElementIndex - The dimension index in the underlying RigidBody that the accumulator owns.
        */
        ElementaryAccumulatorConstraintData(
            const agx::Real& length, const agx::Real& area,
            const agx::Real& fluidVolume,
            const agx::UInt8& accumulatorElementIndex);


        /**
        \internal
        Update to use a new data store.
        */
        void reconfigure(
            const agx::Real& length, const agx::Real& area,
            const agx::Real& fluidVolume,
            const agx::UInt8& accumulatorElementIndex);

        agx::Real length() const;
        agx::Real area() const;
        agx::Real maxVolume() const;
        agx::Real fluidVolume() const;
        agx::UInt8 accumulatorElementIndex() const;

        /**
        \internal
        \return How far out of range the volume is.
                 volume - maxVolume    if volume > maxVolume
                 volume                if volume < 0
                 0                     otherwise

                So a positive value means too much fluid and a negative value
                means that the accumulator is less than empty.
         */
        agx::Real overrun() const;

        /**
        \internal
        \return The distance the spring has been pushed or pulled off the rest
                 state, i.e., the distance that the piston has been lifted.
        */
        agx::Real springCompression() const;

      ElementaryAccumulatorConstraintData& operator=(const ElementaryAccumulatorConstraintData&) = delete;

      private:
        const agx::Real* m_length;
        const agx::Real* m_area;
        const agx::Real* m_fluidVolume;
        const agx::UInt8* m_accumulatorElementIndex;

    };



    /**
    \internal
    Elementary constraint that creates a pressure in the accumulator. The more
    fluid there is in the container the higher the pressure will be.
    */
    class AGXHYDRAULICS_EXPORT ElementarySpringConstraint : public agx::ElementaryConstraintNData<1, ElementaryAccumulatorConstraintData>
    {
      public:
        /**
        \internal
        The spring constant defines the relationship between the amount of
        fluid in the accumulator and the pressure required to add more fluid.
        It is expressed in terms of pressure per distance, where the distace is
        how for along the accumulator that the current amount of fluid has
        filled, i.e., current fluid volumed divided by the area of the
        accumulator.

        \param data - Handle to the data store.
        \param springConstant - The pressure generated by the accumulator, in units of pressure per distance.
        */
        ElementarySpringConstraint(const ElementaryAccumulatorConstraintData& data, agx::Real springConstant);

        virtual agx::UInt getJacobian(
            agx::Jacobian6DOFElement* G, agx::UInt numBlocks,
            agx::UInt row, agx::GWriteState::Enum writeState) override;

        virtual agx::UInt getViolation(agx::Real* g, agx::UInt row) override;

        AGXSTREAM_DECLARE_SERIALIZABLE(agxHydraulics::detail::ElementarySpringConstraint);

      protected:
        ElementarySpringConstraint();
        virtual ~ElementarySpringConstraint() {}
    };


    /**
    \internal
    Constraint that ensures the accumulator doesn't becomes fuller than full or
    emptier than empty.
    */
    class AGXHYDRAULICS_EXPORT ElementaryVolumeConstraint : public agx::ElementaryConstraintNData<1, ElementaryAccumulatorConstraintData>
    {
      public:
        ElementaryVolumeConstraint(const ElementaryAccumulatorConstraintData& data);

        virtual agx::UInt getJacobian(
            agx::Jacobian6DOFElement* G, agx::UInt numBlocks,
            agx::UInt row, agx::GWriteState::Enum writeState) override;

        virtual agx::UInt getViolation(agx::Real* g, agx::UInt row) override;

        AGXSTREAM_DECLARE_SERIALIZABLE(agxHydraulics::detail::ElementaryVolumeConstraint);

      protected:
        ElementaryVolumeConstraint();
        virtual ~ElementaryVolumeConstraint() {}
    };





    /**
    \internal
    ConstraintImplementation that contains one each of the two ElementaryConstraints
    declared above.
    */
    class AGXHYDRAULICS_EXPORT AccumulatorConstraintImplementation : public agx::ConstraintImplementation, public agxStream::Serializable
    {
      public:
        AccumulatorConstraintImplementation(agxHydraulics::Accumulator* accumulator);

        virtual void prepare() override;

        ElementarySpringConstraint* getSpringConstraint();
        ElementaryVolumeConstraint* getVolumeConstraint();

        AGXSTREAM_DECLARE_SERIALIZABLE(agxHydraulics::detail::AccumulatorConstraintImplementation);

      protected:
        AccumulatorConstraintImplementation();
        virtual ~AccumulatorConstraintImplementation() {}

      private:
        agxHydraulics::Accumulator* m_accumulator;
        agx::Real m_length;
        agx::Real m_area;
        agx::Real m_fluidVolume;
        agx::UInt8 m_accumulatorElementIndex;
    };



    AGX_DECLARE_POINTER_TYPES(AccumulatorConstraint);

    /**
    \internal
    Constraint that keeps the fluid volume inside the accumulator within bounds
    and adds a pressure that depends on the current fluid volume inside the
    accumulator.
    */
    class AGXHYDRAULICS_EXPORT AccumulatorConstraint : public agx::Constraint
    {
      public:
        AccumulatorConstraint(Accumulator* accumulator);

        /**
        \return The constraint that causes an increase in internal pressure as
                more fluid is added to the Accumulator.
        */
        const ElementarySpringConstraint* getSpringConstraint() const;

        /**
          \return The constraint that keeps the accumulator from over- or underflowing.
        */
        const ElementaryVolumeConstraint* getVolumeConstraint() const;

        virtual void render(agxRender::RenderManager* canvas, float scale) const override;

        AGXSTREAM_DECLARE_SERIALIZABLE(agxHydraulics::detail::AccumulatorConstraint);

      protected:
        AccumulatorConstraint();
        virtual ~AccumulatorConstraint();
        virtual int getNumDOF() const override;

      private:
        AccumulatorConstraintImplementation* m_accumulatorConstraintImplementation;
    };
  }
  /// \endcond
}

#endif
