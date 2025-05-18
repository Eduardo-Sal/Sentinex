// components/TeamSection.jsx
import React from 'react'

const members = [
  {
    name: 'Eduardo Salinas',
    role: 'Computer Engineer',
    avatar: 'https://media.licdn.com/dms/image/v2/D5603AQH5gozN_rvV_w/profile-displayphoto-shrink_400_400/profile-displayphoto-shrink_400_400/0/1730322115757?e=1752105600&v=beta&t=ymj6orZU4SDXuvVuz8cSFGOchclW4JLnT4E1A8PxEJM',
  },
  {
    name: 'Rolando Garza',
    role: 'Electrical Engineer',
    avatar: 'https://media.licdn.com/dms/image/v2/D5603AQGSOZ_Tu5lcrQ/profile-displayphoto-shrink_400_400/B56ZXy64T3GUAk-/0/1743537276643?e=1752105600&v=beta&t=-LwGvSy0_MVa-6RbzCespHpDztkpvQW6tB9ebrd0oNc',
  },
  {
    name: 'Yahir Jasso',
    role: 'Electrical Engineer',
    avatar: 'https://media.licdn.com/dms/image/v2/D5603AQEXFqEs9wxWYA/profile-displayphoto-shrink_400_400/profile-displayphoto-shrink_400_400/0/1698371336408?e=1752105600&v=beta&t=qI0Ya-EN7rdBgfdcVTLTyzeOxSpW4NniNE2qGeCUuNI',
  },
]

export default function TeamSection() {
  return (
    <section className="bg-gray-50 py-16 md:py-32 dark:bg-transparent">
      <div className="mx-auto max-w-5xl border-t px-6">
        {/* header */}
        <span className="text-caption -ml-6 -mt-3.5 block w-max bg-gray-50 px-6 dark:bg-gray-950">
          Team
        </span>

        {/* grid of people */}
        <div className="mt-12 md:mt-24">
          <div className="grid gap-x-6 gap-y-12 sm:grid-cols-2 lg:grid-cols-3">
            {members.map((m, i) => (
              <div key={i} className="group overflow-hidden">
                <img
                  src={m.avatar}
                  alt={m.name}
                  className="h-96 w-full rounded-md object-cover object-top grayscale transition-all duration-500 hover:grayscale-0 group-hover:h-[22.5rem] group-hover:rounded-xl"
                />
                <div className="px-2 pt-2 sm:pb-0 sm:pt-4">
                  <div className="flex justify-between">
                    <h3 className="text-title text-base font-medium transition-all duration-500 group-hover:tracking-wider">
                      {m.name}
                    </h3>
                    <span className="text-xs">_0{i + 1}</span>
                  </div>
                  <div className="mt-1 flex items-center justify-between">
                    <span className="text-muted-foreground inline-block translate-y-6 text-sm opacity-0 transition duration-300 group-hover:translate-y-0 group-hover:opacity-100">
                      {m.role}
                    </span>
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  )
}